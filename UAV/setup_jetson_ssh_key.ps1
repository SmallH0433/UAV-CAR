#requires -Version 7.0
[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

if ($PSVersionTable.PSVersion.Major -lt 7) {
    throw '请使用 PowerShell 7.6.4 运行此脚本。'
}

$sshDir = Join-Path $env:USERPROFILE '.ssh'
$keyPath = Join-Path $sshDir 'jetson_uav_ed25519'
$pubPath = "$keyPath.pub"

New-Item -ItemType Directory -Force -Path $sshDir | Out-Null

if (-not (Test-Path -LiteralPath $keyPath)) {
    & ssh-keygen.exe -t ed25519 -f $keyPath -C "$env:USERNAME@jetson-uav" -N ''
}

$publicKey = (Get-Content -LiteralPath $pubPath -Raw).Trim()

Write-Host '接下来 SSH 会要求输入一次 Jetson 密码；密码不会写入脚本。'
$remoteCommand = "umask 077; mkdir -p ~/.ssh; touch ~/.ssh/authorized_keys; grep -qxF '$publicKey' ~/.ssh/authorized_keys || printf '%s\n' '$publicKey' >> ~/.ssh/authorized_keys; chmod 700 ~/.ssh; chmod 600 ~/.ssh/authorized_keys"
& ssh.exe jetson-uav $remoteCommand

if ($LASTEXITCODE -ne 0) {
    throw "SSH 密钥安装失败，退出码：$LASTEXITCODE"
}

Write-Host "密钥已配置：$keyPath"
Write-Host '后续可直接执行：ssh -i ~/.ssh/jetson_uav_ed25519 jetson-uav'
