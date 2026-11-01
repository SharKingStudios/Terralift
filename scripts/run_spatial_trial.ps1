[CmdletBinding()]
param(
    [string]$Distro = "Ubuntu-24.04",
    # Internal: set by the launcher when it re-runs itself in a popup console.
    [switch]$InPopup,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$TrialArgs
)

# Windows entry point for the Bash-based experiment launcher.  This avoids the
# common mistake of trying to execute a .sh file directly in PowerShell.
# GUI runs open their own console window so the trial number, ETA, prompts and
# AI responses are visible.  Headless runs stay in the calling terminal.
if (-not $InPopup -and ($TrialArgs -notcontains "--headless")) {
    $quoted = ($TrialArgs | ForEach-Object { '"' + ($_ -replace '"', '\"') + '"' }) -join " "
    $argList = "-NoProfile -NoExit -ExecutionPolicy Bypass -File `"$PSCommandPath`" -Distro `"$Distro`" -InPopup $quoted"
    Start-Process -FilePath "powershell.exe" -ArgumentList $argList
    exit 0
}

$repoWindowsPath = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
if ($repoWindowsPath -notmatch '^(?<drive>[A-Za-z]):\\(?<tail>.*)$') {
    throw "The wrapper supports repositories on a locally mounted Windows drive, not '$repoWindowsPath'. Open WSL and run the Bash launcher directly."
}
$repoWslPath = "/mnt/$($Matches.drive.ToLower())/$($Matches.tail -replace '\\', '/')"

# WSL's native working-directory option avoids all PowerShell-to-Bash quoting
# and Windows-path conversion ambiguity. `bash` receives trial options directly.
if ($InPopup) {
    $log = Join-Path $env:TEMP "spatial_trial_last.log"
    & wsl.exe -d $Distro --cd $repoWslPath -- bash ./scripts/run_spatial_trial.sh @TrialArgs 2>&1 | Tee-Object -FilePath $log
    Write-Host "`nTrial finished. This window can be closed."
    exit $LASTEXITCODE
}
& wsl.exe -d $Distro --cd $repoWslPath -- bash ./scripts/run_spatial_trial.sh @TrialArgs
exit $LASTEXITCODE
