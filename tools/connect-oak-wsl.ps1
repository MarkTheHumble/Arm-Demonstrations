<#
    .DESCRIPTION
        Automatically binds and attaches OAK USB cameras to WSL using usbipd.
        Polls for new devices by VID to solve disconnection issues caused by changing PIDs.
    .NOTES
        Author: nathanrsxtn
#>

# Elevate with sudo or Start-Process
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = New-Object Security.Principal.WindowsPrincipal($identity)
$admin = $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $admin) {
    $argList = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $PSCommandPath)
    $sudo = Get-Command sudo -ErrorAction SilentlyContinue
    if ($sudo) { sudo powershell @argList }
    else { Start-Process powershell -Verb RunAs -ArgumentList $argList }
    exit
}

# Poll for device
Write-Host "Polling for VID_03E7"
while ($true) {
    $devices = (usbipd state | ConvertFrom-Json).Devices |
        Where-Object { $_.InstanceId -match "VID_03E7" -and -not [string]::IsNullOrWhiteSpace($_.BusId) }

    foreach ($d in $devices) {

        if (-not $d.PersistedGuid) {
            usbipd bind --force --busid $d.BusId
            if ($LASTEXITCODE -eq 0) { Write-Host "bind success: $($d.Description)" }
        }

        if (-not $d.ClientIPAddress) {
            usbipd attach --wsl --busid $d.BusId
            if ($LASTEXITCODE -eq 0) { Write-Host "attach success: $($d.Description)" }
        }
    }

    Start-Sleep -Milliseconds 500
}
