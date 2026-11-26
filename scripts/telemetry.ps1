# Windows launcher. Prefer the project's virtual environment. The local Codex
# runtime can also use the dependencies installed for the verified checkout.
$root = Split-Path -Parent $PSScriptRoot
$venvPython = Join-Path $root '.venv/Scripts/python.exe'
$bundledPython = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
$bundle = Join-Path $root 'build/python-deps'
$savedPath = $env:PYTHONPATH
try {
    if (Test-Path -LiteralPath $venvPython) {
        $interpreter = $venvPython
    } elseif ((Test-Path -LiteralPath $bundledPython) -and (Test-Path -LiteralPath $bundle)) {
        $interpreter = $bundledPython
        $env:PYTHONPATH = if ($savedPath) { "$bundle;$savedPath" } else { $bundle }
    } elseif (Get-Command py -ErrorAction SilentlyContinue) {
        $interpreter = 'py'
    } else {
        $interpreter = 'python'
    }
    & $interpreter (Join-Path $root 'tools/telemetry.py') @args
    $result = $LASTEXITCODE
} finally {
    $env:PYTHONPATH = $savedPath
}
exit $result
