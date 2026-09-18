"""
testbench/aws_client.py
AWS Client & API-Gateway Client für die HIL-Testsuite.
Verwendet Auth0 Bearer-Tokens für offizielle API-Gateway-Endpunkte (/activatedevice, /uploads),
verifiziert Zustände und Payloads in DynamoDB (iotCatalog & iotPayload)
und steuert OTA-Triggers über AWS IoT Named Shadows.
"""

import os
import io
import time
import json
import urllib.parse
import boto3
from botocore.exceptions import ClientError
from . import config
from .privacy import mask_uid, mask_s3_key, mask_topic, mask_path, sanitize_log_line, register_github_mask

try:
    import requests
except ImportError:
    requests = None

try:
    from PIL import Image, ImageDraw
except ImportError:
    Image = None
    ImageDraw = None


class AWSTestVerifier:
    def __init__(self, region=None, endpoint_url=None):
        self.region = region or config.AWS_REGION
        self.endpoint_url = endpoint_url or config.AWS_IOT_ENDPOINT
        self.dynamodb = boto3.resource("dynamodb", region_name=self.region)
        self.s3 = boto3.client("s3", region_name=self.region)
        self.iot_data = boto3.client("iot-data", region_name=self.region, endpoint_url=self.endpoint_url)
        self.catalog_table = self.dynamodb.Table(config.DYNAMODB_CATALOG_TABLE)
        self.payload_table = self.dynamodb.Table(config.DYNAMODB_PAYLOAD_TABLE)

        # Offizielle REST-API Endpunkte
        self.api_base_url = os.environ.get("HIL_API_BASE_URL", "").rstrip("/")
        self.upload_api_url = f"{self.api_base_url}/uploads" if self.api_base_url else ""
        self.activate_api_url = f"{self.api_base_url}/activatedevice" if self.api_base_url else ""
        self.s3_bucket = os.environ.get("HIL_S3_BUCKET", "")

        # Auth0 Token Cache (In-Memory + Persistenter Disk-Cache)
        self.cache_dir = config.CACHE_DIR
        self.token_cache_file = os.path.join(self.cache_dir, ".auth0_token_cache.json")
        self._cached_auth0_token = None
        self._token_expiry = 0

    def invalidate_auth0_cache(self):
        """Löscht das gecachte Auth0-Token aus dem RAM und vom Dateisystem."""
        self._cached_auth0_token = None
        self._token_expiry = 0
        if os.path.isfile(self.token_cache_file):
            try:
                os.remove(self.token_cache_file)
                print("🗑️ Auth0-Token-Cache auf der Festplatte invalidiert.")
            except Exception as e:
                print(f"⚠️ Konnte {self.token_cache_file} nicht löschen: {e}")

    def get_auth0_token(self):
        """
        Holt ein Auth0 Bearer-Token über den Client Credentials Grant.
        Prüft zuerst den In-Memory- und den persistenten Disk-Cache (.auth0_token_cache.json) mit TTL.
        """
        now = time.time()

        # 1. In-Memory Cache prüfen
        if self._cached_auth0_token and now < self._token_expiry:
            return self._cached_auth0_token

        # 2. Persistenter Disk-Cache prüfen
        if os.path.isfile(self.token_cache_file):
            try:
                with open(self.token_cache_file, "r", encoding="utf-8") as f:
                    cache_data = json.load(f)
                cached_token = cache_data.get("access_token")
                expires_at = cache_data.get("expires_at", 0)

                # Gültig mit 5 Minuten (300s) Sicherheitsabstand
                if cached_token and now < (expires_at - 300):
                    self._cached_auth0_token = cached_token
                    self._token_expiry = expires_at - 60
                    exp_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(expires_at))
                    remaining_days = round((expires_at - now) / 86400, 1)
                    print(f"📦 Verwende gecachtes Auth0 Bearer-Token (gültig bis {exp_str} / ~{remaining_days} Tage).")
                    return cached_token
            except Exception as e:
                print(f"⚠️ Konnte Auth0-Cache-Datei nicht laden ({e}), fordere neues Token an.")

        # 3. Neues Token von Auth0 beziehen
        if requests is None:
            raise RuntimeError("Das Modul 'requests' ist nicht installiert. Bitte 'pip install requests' ausführen.")

        client_id = os.environ.get("AUTH0_CLIENT_ID")
        client_secret = os.environ.get("AUTH_0_CLIENT_SECRET")
        audience = os.environ.get("AUTH_0_AUDIENCE", "localhost:3000/")
        grant_type = os.environ.get("AUTH_0_GRANT_TYPE", "client_credentials")
        domain = os.environ.get("AUTH0_DOMAIN")

        if not client_id or not client_secret:
            raise ValueError(
                "Auth0-Zugangsdaten nicht gefunden! Bitte AUTH0_CLIENT_ID und AUTH_0_CLIENT_SECRET in .env setzen."
            )
        if not domain:
            raise ValueError(
                "AUTH0_DOMAIN nicht gefunden! Bitte AUTH0_DOMAIN in .env setzen (z. B. your-tenant.eu.auth0.com)."
            )

        token_url = f"https://{domain}/oauth/token"
        payload = {
            "client_id": client_id,
            "client_secret": client_secret,
            "audience": audience,
            "grant_type": grant_type
        }

        print(f"🔑 Fordere neues Auth0 Bearer-Token an von {domain}...")
        resp = requests.post(token_url, json=payload, timeout=10)
        if resp.status_code != 200:
            raise RuntimeError(f"Auth0 Token-Anfrage fehlgeschlagen (Status {resp.status_code}): {resp.text}")

        data = resp.json()
        token = data.get("access_token")
        expires_in = data.get("expires_in", 3600)

        if not token:
            raise RuntimeError(f"Kein access_token in Auth0-Antwort gefunden: {data}")

        expires_at = now + expires_in
        self._cached_auth0_token = token
        self._token_expiry = expires_at - 60

        # Disk-Cache speichern
        try:
            os.makedirs(self.cache_dir, exist_ok=True)
            with open(self.token_cache_file, "w", encoding="utf-8") as f:
                json.dump({
                    "access_token": token,
                    "expires_at": expires_at,
                    "cached_at": now
                }, f, indent=2)
            print(f"💾 Auth0-Token erfolgreich im Disk-Cache gespeichert: {mask_path(self.token_cache_file)}")
        except Exception as e:
            print(f"⚠️ Konnte Auth0-Token nicht im Disk-Cache speichern: {e}")

        print(f"✅ Auth0 Token erfolgreich erhalten (Gültigkeit: {expires_in}s / {round(expires_in/86400, 1)} Tage).")
        return token

    def _get_api_headers(self):
        """Erzeugt Authorization- und Content-Type-Header für API Gateway Requests."""
        token = self.get_auth0_token()
        return {
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json"
        }

    def get_device_status(self, device_id):
        """
        Liest den Gerätestatus direkt aus der DynamoDB-Tabelle iotCatalog.
        Verwendet Query auf den Partition Key 'serialNumber'.
        """
        keys_to_try = [str(device_id)]
        if "-" in device_id:
            raw_mac = device_id.split("-", 1)[1]
            keys_to_try.extend([raw_mac, raw_mac.upper(), raw_mac.lower()])
        else:
            keys_to_try.extend([f"epd7-{device_id}", f"epd13-{device_id}", device_id.upper()])

        for k in keys_to_try:
            try:
                res = self.catalog_table.query(
                    KeyConditionExpression="serialNumber = :s",
                    ExpressionAttributeValues={":s": k},
                    Limit=1
                )
                items = res.get("Items", [])
                if items:
                    return items[0]
            except ClientError as e:
                print(f"⚠️ DynamoDB Query Fehler für Key '{k}': {e}")
        return None

    def set_device_activation_pending(self, device_id, organization_name="HIL-Test-Org"):
        """
        Startet die Aktivierung über die offizielle REST-API:
        POST /activatedevice mit Auth0 Bearer-Token.
        """
        if not self.activate_api_url:
            raise ValueError("HIL_API_BASE_URL ist nicht gesetzt! Bitte in .env oder Umgebungsvariablen definieren.")

        target_name = device_id if device_id.startswith(("epd7-", "epd13-")) else f"epd7-{device_id}"
        register_github_mask(target_name)
        register_github_mask(device_id)
        print(f"📡 Rufe Aktivierungs-API auf für '{mask_uid(target_name)}' (POST /activatedevice)...")

        headers = self._get_api_headers()
        payload = {
            "deviceName": target_name,
            "enable": True,
            "organizationName": organization_name
        }

        resp = requests.post(self.activate_api_url, json=payload, headers=headers, timeout=12)
        if resp.status_code == 401:
            self.invalidate_auth0_cache()
            raise RuntimeError("Aktivierungs-API nicht autorisiert (401 Unauthorized): Token abgelaufen oder ungültig.")
        if resp.status_code != 200:
            raise RuntimeError(f"Aktivierungs-API fehlgeschlagen (Status {resp.status_code}): {resp.text}")

        res_json = resp.json()
        print(f"✅ Aktivierungs-API Antwort: {res_json.get('message', res_json)}")
        return res_json

    def wait_for_activation(self, device_id, timeout=30):
        """Wartet, bis das Gerät in DynamoDB iotCatalog als aktiviert bestätigt ist."""
        start = time.time()
        print(f"⏳ Warte auf DynamoDB-Aktivierung von '{mask_uid(device_id)}' (Timeout: {timeout}s)...")

        while (time.time() - start) < timeout:
            item = self.get_device_status(device_id)
            if item:
                status = item.get("activation_status") or item.get("status")
                is_active = item.get("activated", False)
                if status in ("activated", "active") or is_active is True:
                    elapsed = round(time.time() - start, 2)
                    print(f"✅ Gerät '{mask_uid(device_id)}' in DynamoDB als aktiv bestätigt ({elapsed}s).")
                    return item
            time.sleep(2)

        raise TimeoutError(f"Gerät '{mask_uid(device_id)}' wurde nicht innerhalb von {timeout}s in DynamoDB aktiviert!")

    def generate_test_image(self, width=800, height=480, label="HIL TEST IMAGE"):
        """
        Erzeugt ein Testbild mit Timestamp und Farbfeldern der 7-Farben E-Paper Palette
        und gibt es als JPEG-Bytes zurück.
        """
        if Image is None or ImageDraw is None:
            raise RuntimeError("Pillow ist nicht installiert. Bitte 'pip install pillow' ausführen.")

        img = Image.new("RGB", (width, height), color=(255, 255, 255))
        draw = ImageDraw.Draw(img)

        # Rahmen
        draw.rectangle([(10, 10), (width - 10, height - 10)], outline=(0, 0, 0), width=4)

        # Textzeilen
        ts_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        draw.text((30, 30), f"PAPERLESSPAPER HIL TEST", fill=(0, 0, 0))
        draw.text((30, 60), f"Gerät: {label} | Zeit: {ts_str}", fill=(200, 0, 0))

        # 7-Farben E-Paper Palette Blöcke
        colors = [
            ((0, 0, 0), "Schwarz"),
            ((255, 0, 0), "Rot"),
            ((0, 0, 255), "Blau"),
            ((255, 255, 0), "Gelb"),
            ((0, 200, 0), "Grün"),
            ((255, 128, 0), "Orange"),
        ]
        block_w = max(40, (width - 80) // len(colors))
        block_h = min(120, height // 3)
        y_top = 110

        for idx, (col, col_name) in enumerate(colors):
            x0 = 40 + idx * block_w
            y0 = y_top
            x1 = x0 + block_w - 10
            y1 = y0 + block_h
            draw.rectangle([(x0, y0), (x1, y1)], fill=col, outline=(0, 0, 0), width=2)
            draw.text((x0 + 5, y1 + 10), col_name, fill=(0, 0, 0))

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=95)
        return buf.getvalue()

    @staticmethod
    def _extract_bucket_from_url(url):
        """Extrahiert den S3-Bucket-Namen sicher aus Path-Style oder Virtual-Hosted-Style URLs."""
        try:
            parsed_u = urllib.parse.urlparse(url)
            parts = parsed_u.path.lstrip("/").split("/", 1)
            if "s3" in parsed_u.netloc and len(parts) > 1:
                return parts[0]
            elif ".s3." in parsed_u.netloc or parsed_u.netloc.endswith(".s3.amazonaws.com"):
                return parsed_u.netloc.split(".s3", 1)[0]
        except Exception:
            pass
        return None

    def get_signed_upload_url(self, device_id):
        """
        Fordert eine Presigned S3-Upload-URL ausschließlich über die offizielle REST-API an:
        POST /uploads mit Auth0 Bearer-Token.
        """
        if not self.upload_api_url:
            raise ValueError("HIL_API_BASE_URL ist nicht gesetzt! Bitte in .env oder Umgebungsvariablen definieren.")

        target_name = device_id if device_id.startswith(("epd7-", "epd13-")) else f"epd7-{device_id}"
        register_github_mask(target_name)
        register_github_mask(device_id)
        print(f"📡 Fordere Presigned Upload-URL an über REST-API (POST /uploads für '{mask_uid(target_name)}')...")

        headers = self._get_api_headers()
        payload = {"deviceName": target_name}

        resp = requests.post(self.upload_api_url, json=payload, headers=headers, timeout=12)
        if resp.status_code == 401:
            self.invalidate_auth0_cache()
            raise RuntimeError("Upload-API nicht autorisiert (401 Unauthorized): Token abgelaufen oder ungültig.")
        if resp.status_code != 200:
            raise RuntimeError(f"Upload-API fehlgeschlagen (Status {resp.status_code}): {resp.text}")

        data = resp.json()
        if not data.get("success") or "uploadURL" not in data:
            raise RuntimeError(f"Ungültige Antwort von Upload-API: {data}")

        upload_url = data["uploadURL"]
        key = data.get("Key", f"epdPicture-{target_name}.jpg")

        # S3-Bucket dynamisch aus Presigned URL ermitteln, falls nicht per Env vorgegeben
        if not self.s3_bucket:
            self.s3_bucket = self._extract_bucket_from_url(upload_url) or ""

        print(f"✅ Presigned Upload-URL erfolgreich über API erhalten: {mask_s3_key(key)}")
        return upload_url, key

    def upload_image_via_signed_url(self, upload_url, image_bytes):
        """Lädt JPEG-Bilddaten per HTTP PUT an die signierte S3-URL hoch."""
        resp = requests.put(
            upload_url,
            data=image_bytes,
            headers={"Content-Type": "image/jpeg"},
            timeout=30
        )
        resp.raise_for_status()
        print(f"✅ Bild erfolgreich per HTTP PUT an S3 hochgeladen ({len(image_bytes)} Bytes).")
        return True

    def upload_test_image(self, device_id, width=800, height=480):
        """
        Führt den vollständigen Bild-Upload-Flow aus:
        1. Presigned Upload-URL über die offizielle API (POST /uploads mit Auth0) anfordern
        2. Testbild generieren
        3. Per HTTP PUT an die S3-URL hochladen
        4. Alten .gz-Cache in S3 bereinigen
        """
        upload_url, key = self.get_signed_upload_url(device_id)
        img_bytes = self.generate_test_image(width=width, height=height, label=device_id)

        print(f"📤 Lade Bild an Presigned S3-URL hoch...")
        self.upload_image_via_signed_url(upload_url, img_bytes)

        # Alten komprimierten Cache löschen, falls vorhanden
        if self.s3_bucket:
            gz_key = key.replace(".jpg", ".gz")
            try:
                self.s3.delete_object(Bucket=self.s3_bucket, Key=gz_key)
                print(f"🧹 Alten GZ-Cache '{gz_key}' aus S3 entfernt.")
            except Exception:
                pass

        return key, int(time.time())

    def deactivate_device(self, device_id):
        """
        Deaktiviert das Gerät über die offizielle REST-API:
        POST /activatedevice mit Auth0 Bearer-Token und reset: True.
        Bereinigt zudem Bilddaten in S3.
        """
        if not self.activate_api_url:
            raise ValueError("HIL_API_BASE_URL ist nicht gesetzt! Bitte in .env oder Umgebungsvariablen definieren.")

        target_name = device_id if device_id.startswith(("epd7-", "epd13-")) else f"epd7-{device_id}"
        print(f"\n🛑 Deaktiviere Gerät über REST-API (POST /activatedevice mit reset: True)...")

        headers = self._get_api_headers()
        payload = {
            "deviceName": target_name,
            "reset": True
        }

        resp = requests.post(self.activate_api_url, json=payload, headers=headers, timeout=12)
        if resp.status_code == 401:
            self.invalidate_auth0_cache()
            raise RuntimeError("Deaktivierungs-API nicht autorisiert (401 Unauthorized): Token abgelaufen oder ungültig.")
        if resp.status_code != 200:
            raise RuntimeError(f"Deaktivierungs-API fehlgeschlagen (Status {resp.status_code}): {resp.text}")

        res_json = resp.json()
        print(f"✅ Deaktivierungs-API Antwort: {res_json.get('message', res_json)}")

        # S3 Testbilder bereinigen
        if self.s3_bucket:
            for ext in (".jpg", ".gz"):
                key = f"epdPicture-{target_name}{ext}"
                try:
                    self.s3.delete_object(Bucket=self.s3_bucket, Key=key)
                    print(f"🗑️ S3-Objekt '{key}' bereinigt.")
                except Exception:
                    pass

        return True

    def wait_for_payload_ack(self, device_id, min_timestamp=None, timeout=60):
        """
        Prüft in DynamoDB iotPayload, ob die Bestätigung (EventType 'activate' oder 'update')
        nach dem Bild-Rendering korrekt abgelegt wurde.
        """
        start = time.time()
        min_ts = (min_timestamp or int(time.time())) * 1000 - 15000  # ms mit Puffer
        print(f"⏳ Warte auf Quittierungs-Payload in DynamoDB iotPayload für '{mask_uid(device_id)}' (Timeout: {timeout}s)...")

        dev_candidates = [str(device_id)]
        if not device_id.startswith(("epd7-", "epd13-")):
            dev_candidates.extend([f"epd7-{device_id}", f"epd13-{device_id}"])
        else:
            dev_candidates.append(device_id.split("-", 1)[1])

        while (time.time() - start) < timeout:
            for cand in dev_candidates:
                try:
                    res = self.payload_table.query(
                        KeyConditionExpression="DeviceId = :dev_id",
                        ExpressionAttributeValues={":dev_id": cand},
                        ScanIndexForward=False,
                        Limit=5
                    )
                    items = res.get("Items", [])
                    for it in items:
                        ts = int(it.get("EventTimestamp") or it.get("AwsTimestamp") or 0)
                        if ts >= min_ts:
                            msg = str(it.get("EventMessage", ""))
                            ev_type = str(it.get("EventType", ""))
                            status = str(it.get("status", ""))
                            if "update" in ev_type.lower() or "update_ok" in msg or "ok" in status.lower() or "activate" in ev_type.lower():
                                elapsed = round(time.time() - start, 2)
                                clean_msg = sanitize_log_line(msg)
                                print(f"✅ Quittierungs-Payload in DynamoDB iotPayload verifiziert ({elapsed}s): Typ={ev_type}, Message={clean_msg}")
                                return it
                except ClientError as e:
                    print(f"⚠️ DynamoDB iotPayload Query Warnung für {mask_uid(cand)}: {e}")

            time.sleep(2)

        raise TimeoutError(f"Keine Bild-Quittierung für '{device_id}' in DynamoDB iotPayload gefunden!")

    def upload_candidate_firmware(self, device_id, bin_path, key_name=None, bucket=None):
        """
        Lädt eine gebuildete Test-Firmware-Binärdatei sicher in den S3-Bucket hoch.
        Strikte Sicherheits-Garantie:
          - Key MUSS mit 'test-firmware-' beginnen.
          - Key darf KEINESFALLS in config.PROTECTED_S3_KEYS enthalten sein.
          - Wird mit ACL 'public-read' bereitgestellt, damit das ESP32 per HTTP zugreifen kann.
        Gibt die direkte Download-URL (z. B. 'http://<bucket-domain>/test-firmware-epd7.bin') und den Key zurück.
        """
        target_bucket = bucket or config.HIL_OTA_BUCKET
        if not target_bucket:
            raise ValueError("Kein S3-Bucket für OTA Firmware-Upload definiert (HIL_OTA_BUCKET)!")

        if not os.path.isfile(bin_path):
            raise FileNotFoundError(f"Kandidaten-Firmware nicht gefunden unter: {bin_path}")

        file_size = os.path.getsize(bin_path)
        if file_size < 100_000:
            raise ValueError(f"Firmware-Datei '{bin_path}' ist auffällig klein ({file_size} Bytes) - Abbruch zur Sicherheit.")

        if not key_name:
            dtype = "epd13" if "13" in str(device_id) else "epd7"
            key_name = f"test-firmware-{dtype}.bin"

        # Sicherheitsprüfungen
        if not key_name.startswith("test-firmware-"):
            raise ValueError(
                f"SICHERHEITSVERLETZUNG: S3-Key '{key_name}' muss mit 'test-firmware-' beginnen! "
                "Produktivdateien dürfen niemals überschrieben werden."
            )
        if key_name in config.PROTECTED_S3_KEYS:
            raise ValueError(
                f"SICHERHEITSVERLETZUNG: '{key_name}' ist eine geschützte Produktionsdatei! "
                "Aktion wurde blockiert."
            )

        print(f"\n📤 [S3 OTA] Lade Kandidaten-Firmware ({round(file_size/1024, 1)} KB) an S3 hoch...")
        print(f"   ➔ Bucket: {target_bucket}")
        print(f"   ➔ Key:    {key_name}")

        self.s3.upload_file(
            Filename=bin_path,
            Bucket=target_bucket,
            Key=key_name,
            ExtraArgs={"ACL": "public-read", "ContentType": "application/octet-stream"}
        )

        direct_url = f"http://{target_bucket}/{key_name}"
        print(f"✅ [S3 OTA] Test-Firmware erfolgreich hochgeladen: {direct_url}")
        return direct_url, key_name

    def cleanup_candidate_firmware(self, key_name, bucket=None):
        """Löscht die temporäre Test-Firmware-Binärdatei wieder aus dem S3-Bucket."""
        target_bucket = bucket or config.HIL_OTA_BUCKET
        if not key_name.startswith("test-firmware-") or key_name in config.PROTECTED_S3_KEYS:
            print(f"⚠️ [S3 OTA] Überspringe Bereinigung von unsicherem/geschütztem Key: {key_name}")
            return False

        try:
            self.s3.delete_object(Bucket=target_bucket, Key=key_name)
            print(f"🗑️ [S3 OTA] Temporäre Test-Firmware '{key_name}' erfolgreich aus Bucket '{target_bucket}' gelöscht.")
            return True
        except Exception as e:
            print(f"⚠️ [S3 OTA] Fehler beim Löschen von '{key_name}' aus Bucket '{target_bucket}': {e}")
            return False

    def trigger_ota(self, device_id, ota_url):
        """
        Triggert ein OTA-Update (Manifest JSON oder direkte Binär-URL) sowohl über
        den AWS IoT Named Shadow 'settings' (reported.otaUrl) als auch per MQTT-Publish
        auf '$aws/things/{device_id}/epaper/receive' (doc['ota']).
        """
        thing_candidates = [str(device_id)]
        if not device_id.startswith(("epd7-", "epd13-")):
            thing_candidates.extend([f"epd7-{device_id}", f"epd13-{device_id}"])

        last_error = None
        for thing in thing_candidates:
            # 1. AWS IoT Named Shadow 'settings' setzen
            shadow_payload = {
                "state": {
                    "reported": {
                        "otaUrl": ota_url
                    }
                }
            }
            try:
                self.iot_data.update_thing_shadow(
                    thingName=thing,
                    shadowName="settings",
                    payload=json.dumps(shadow_payload)
                )
                print(f"📡 [OTA] Shadow 'settings.otaUrl' für '{mask_uid(thing)}' gesetzt: {ota_url}")
            except ClientError as e:
                err_code = e.response.get("Error", {}).get("Code")
                if err_code != "ResourceNotFoundException" or thing == thing_candidates[-1]:
                    print(f"⚠️ Warnung beim Setzen des IoT Shadows für {mask_uid(thing)}: {e}")
                    last_error = e

            # 2. Direkt per MQTT an epaper/receive publishen (falls Gerät bereits online/wach ist)
            mqtt_topic = f"$aws/things/{thing}/epaper/receive"
            mqtt_payload = json.dumps({"ota": ota_url})
            try:
                self.iot_data.publish(
                    topic=mqtt_topic,
                    qos=1,
                    payload=mqtt_payload
                )
                print(f"📡 [OTA] MQTT-Nachricht auf '{mask_topic(mqtt_topic)}' gesendet: {mqtt_payload}")
                return True
            except ClientError as e:
                print(f"⚠️ Fehler beim MQTT-Publish auf {mqtt_topic}: {e}")
                last_error = e

        if last_error:
            raise last_error
        return True

    def trigger_ota_via_shadow(self, device_id, ota_manifest_url):
        """Kompatibilitäts-Alias für trigger_ota."""
        return self.trigger_ota(device_id, ota_manifest_url)
