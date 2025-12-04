import os
Import("env")

def apply_env_vars():
    try:
        if not os.path.isfile(".env"):
            print("Warning: .env file not found.")
            return

        with open(".env") as f:
            for line in f:
                if line.strip() and not line.startswith("#"):
                    parts = line.strip().split("=", 1)
                    if len(parts) == 2:
                        key = parts[0].strip()
                        value = parts[1].strip()
                        # 1. REMOVE OUTER QUOTES 
                        if (value.startswith('"') and value.endswith('"')) or \
                           (value.startswith("'") and value.endswith("'")):
                            value = value[1:-1]
                        # 2. ESCAPE NEWLINES (Vital for the Cert)
                        value = value.replace(r"\n", r"\\n")
                        # 3. ESCAPE INTERNAL QUOTES
                        value = value.replace(r'"', r'\"')

                        env.Append(CPPDEFINES=[
                            (key, f'\\"{value}\\"')
                        ])

    except Exception as e:
        print(f"Error processing .env: {e}")

apply_env_vars()