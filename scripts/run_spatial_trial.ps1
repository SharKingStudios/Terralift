[CmdletBinding()]
param(
    [string]$Distro = "Ubuntu-24.04",
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$TrialArgs
)

# Windows entry point for the Bash-based experiment launcher.  This avoids the
# common mistake of trying to execute a .sh file directly in PowerShell.
$repoWindowsPath = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
if ($repoWindowsPath -notmatch '^(?<drive>[A-Za-z]):\\(?<tail>.*)$') {
    throw "The wrapper supports repositories on a locally mounted Windows drive, not '$repoWindowsPath'. Open WSL and run the Bash launcher directly."
}
$repoWslPath = "/mnt/$($Matches.drive.ToLower())/$($Matches.tail -replace '\\', '/')"

# WSL's native working-directory option avoids all PowerShell-to-Bash quoting
# and Windows-path conversion ambiguity. `bash` receives trial options directly.
& wsl.exe -d $Distro --cd $repoWslPath -- bash ./scripts/run_spatial_trial.sh @TrialArgs
exit $LASTEXITCODE
