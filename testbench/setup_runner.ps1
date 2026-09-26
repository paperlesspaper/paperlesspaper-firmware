# ==============================================================================
# testbench/setup_runner.ps1
# Setup- & Einrichtungs-Skript für den Windows HIL Self-Hosted GitHub Runner
# ==============================================================================

Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "🔧 EINRICHTUNG DES HIL TESTBENCH RUNNERS (WINDOWS)" -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan

# 1. Python Überprüfung
Write-Host "`n[1/4] Prüfe Python Installation..." -ForegroundColor Yellow
try {
    $pyVersion = python --version
    Write-Host "✅ Gefunden: $pyVersion" -ForegroundColor Green
} catch {
    Write-Host "❌ Python nicht im PATH gefunden! Bitte Python 3.10+ installieren." -ForegroundColor Red
    exit 1
}

# 2. Abhängigkeiten installieren
Write-Host "`n[2/4] Installiere Testbench Python-Pakete (pyserial, esptool, boto3, pytest)..." -ForegroundColor Yellow
$reqFile = Join-Path $PSScriptRoot "requirements.txt"
if (Test-Path $reqFile) {
    pip install -r $reqFile
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✅ Alle Python-Pakete erfolgreich installiert." -ForegroundColor Green
    } else {
        Write-Host "❌ Fehler bei der Paketinstallation!" -ForegroundColor Red
    }
} else {
    Write-Host "⚠️ requirements.txt nicht gefunden: $reqFile" -ForegroundColor Yellow
}

# 3. COM-Ports auflisten
Write-Host "`n[3/4] Erkannte serielle COM-Ports:" -ForegroundColor Yellow
python -c "import serial.tools.list_ports as lp; ports = lp.comports(); print('\n'.join([f'  - {p.device}: {p.description}' for p in ports]) if ports else '  (Keine COM-Ports gefunden)')"

# 4. GitHub Runner Anleitung
Write-Host "`n[4/4] GitHub Actions Self-Hosted Runner Registrierung:" -ForegroundColor Yellow
Write-Host "Führe folgende Schritte im Zielverzeichnis (z. B. C:\actions-runner) aus:" -ForegroundColor White
Write-Host @"
1. Runner von GitHub herunterladen & entpacken (Repo Settings -> Actions -> Runners -> New runner)
2. Runner konfigurieren mit den Labels:
   ./config.cmd --url <REPO_URL> --token <TOKEN> --labels self-hosted,Windows,X64,hil-testbench
3. Als Windows-Dienst installieren (optional für Dauerbetrieb):
   ./run.cmd oder .\svc.cmd install ; .\svc.cmd start
"@ -ForegroundColor Cyan

Write-Host "`n🎉 Setup-Skript abgeschlossen. Du kannst die Testbench nun mit folgendem Befehl testen:" -ForegroundColor Green
Write-Host "   python -m testbench.run_testbench --list-ports" -ForegroundColor White
Write-Host "============================================================" -ForegroundColor Cyan
