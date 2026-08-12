#requires -Version 7.0
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$SourcePath,

    [string]$RemotePath = '/home/jetson/uav'
)

$ErrorActionPreference = 'Stop'

if ($PSVersionTable.PSVersion.Major -lt 7) {
    throw '请使用 PowerShell 7.6.4 运行此脚本。'
}

$resolvedSource = (Resolve-Path -LiteralPath $SourcePath).Path
$keyPath = Join-Path $env:USERPROFILE '.ssh\jetson_uav_ed25519'

if (-not (Test-Path -LiteralPath $keyPath)) {
    throw "未找到 SSH 密钥，请先运行 setup_jetson_ssh_key.ps1。"
}

Write-Host "同步指定目录：$resolvedSource -> jetson-uav:$RemotePath"
Write-Host '本脚本不会删除 Jetson 上的其他文件，也不会使用 --delete。'

& scp.exe -r -i $keyPath -- $resolvedSource "jetson-uav:$RemotePath"

if ($LASTEXITCODE -ne 0) {
    throw "文件同步失败，退出码：$LASTEXITCODE"
}

Write-Host '同步完成。'
