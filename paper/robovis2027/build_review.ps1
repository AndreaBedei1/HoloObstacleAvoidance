param(
    [string]$Latexmk = "",
    [string]$Python = "python"
)

$ErrorActionPreference = "Stop"
$paperRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$outputDir = Join-Path $paperRoot "review_blind"
$sourcePdf = Join-Path $paperRoot "main.pdf"
$countScript = Join-Path $paperRoot "tools\count_robovis_characters.py"
$auditJson = Join-Path $outputDir "character_count.json"
$shadowBibliography = Join-Path $paperRoot "references.bib"

if (Test-Path -LiteralPath $shadowBibliography) {
    throw "Shadow bibliography found at $shadowBibliography. Remove it so ../references.bib is the only source."
}

if (-not $Latexmk) {
    $latexmkCommand = Get-Command latexmk.exe -ErrorAction SilentlyContinue
    if ($latexmkCommand) {
        $Latexmk = $latexmkCommand.Source
    } else {
        $candidate = Join-Path $env:LOCALAPPDATA "Programs\MiKTeX\miktex\bin\x64\latexmk.exe"
        if (-not (Test-Path -LiteralPath $candidate)) {
            throw "latexmk.exe was not found; pass its path with -Latexmk."
        }
        $Latexmk = $candidate
    }
}

New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
Push-Location $paperRoot
try {
    & $Latexmk -g -pdf -interaction=nonstopmode -halt-on-error -file-line-error main.tex
    if ($LASTEXITCODE -ne 0) {
        throw "LaTeX build failed with exit code $LASTEXITCODE."
    }
} finally {
    Pop-Location
}

Copy-Item -LiteralPath $sourcePdf -Destination (Join-Path $outputDir "main.pdf") -Force
& $Python $countScript (Join-Path $outputDir "main.pdf") --json $auditJson
if ($LASTEXITCODE -ne 0) {
    throw "ROBOVIS character audit failed with exit code $LASTEXITCODE."
}
