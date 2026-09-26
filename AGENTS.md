# Agent Guidelines & Repository Rules

## Git Workflow Rules
- **KEINE SELBSTÄNDIGEN COMMITS ODER PUSHES:** Der Agent darf niemals selbständig `git commit` oder `git push` ausführen. Alle Dateiänderungen verbleiben uncommitted im Arbeitsverzeichnis; Commits und Pushes werden ausschließlich manuell vom Nutzer durchgeführt.
- **STRIKTES VERBOT FÜR MAIN:** Niemals direkt auf den Branch `main` (oder `master`) committen oder pushen.
- **Entwicklungs-Branch:** Alle Änderungen erfolgen ausschließlich auf dem Branch `paper-l` (oder dedizierten Feature-Branches).
- **Merges nach main:** Merges oder Übertragungen nach `main` dürfen niemals automatisiert vom Agenten durchgeführt werden, sondern erfolgen ausschließlich durch den Nutzer (z. B. via Pull Request).
