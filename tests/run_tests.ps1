# ==============================================================================
# Host Unit Test Runner for QFC-1 Flight Controller Firmware
# Standard: ISO 26262-6 Software Unit Verification / MISRA-C Compliance
# Compiler: GCC (MSYS2 / MinGW-w64)
# ==============================================================================
$ErrorActionPreference = "Stop"

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host "  QFC-1 FLIGHT CONTROLLER - HOST UNIT TEST SUITE (GCC)    " -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir

$Includes = @(
    "-I", "$ProjectRoot/firmware/include",
    "-I", "$ProjectRoot/tests",
    "-I", "$ProjectRoot/tests/host_mocks"
)

$CommonFlags = @(
    "-Wall",
    "-Wextra",
    "-Wpedantic",
    "-Wshadow",
    "-Wpointer-arith",
    "-Wcast-qual",
    "-Wstrict-prototypes",
    "-Wmissing-prototypes",
    "-std=c99",
    "-O0",
    "-g"
)

# Discover all test files in tests/ matching test_*.c
$TestFiles = Get-ChildItem -Path "$ScriptDir" -Filter "test_*.c"

if ($TestFiles.Count -eq 0) {
    Write-Host "[!] No unit test suites found in $ScriptDir" -ForegroundColor Yellow
    exit 0
}

$BuildDir = "$ScriptDir/build"
if (-not (Test-Path "$BuildDir")) {
    New-Item -ItemType Directory -Path "$BuildDir" | Out-Null
}

$AllFirmwareSources = Get-ChildItem -Path "$ProjectRoot/firmware/src" -Filter "*.c" -ErrorAction SilentlyContinue | ForEach-Object { $_.FullName }
$AllHostMocks = Get-ChildItem -Path "$ScriptDir/host_mocks" -Filter "*.c" -ErrorAction SilentlyContinue | ForEach-Object { $_.FullName }

$TotalSuites = $TestFiles.Count
$PassedSuites = 0
$FailedSuites = 0

Write-Host "[*] Found $TotalSuites unit test suite(s). Starting execution...`n" -ForegroundColor Yellow

foreach ($TestFile in $TestFiles) {
    $SuiteName = $TestFile.BaseName
    $TestBin = "$BuildDir/$SuiteName.exe"

    Write-Host "----------------------------------------------------------" -ForegroundColor DarkGray
    Write-Host "[*] Compiling & Running Suite: $SuiteName" -ForegroundColor Yellow

    $Sources = @($TestFile.FullName)
    if ($AllFirmwareSources) {
        $Sources += $AllFirmwareSources
    }
    if ($AllHostMocks) {
        $Sources += $AllHostMocks
    }


    # Compile test suite
    $CompileArgs = @()
    $CompileArgs += $CommonFlags
    $CompileArgs += $Includes
    $CompileArgs += $Sources
    $CompileArgs += @("-lm", "-o", "$TestBin")

    & gcc @CompileArgs
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[FAIL] Compilation failed for $SuiteName" -ForegroundColor Red
        $FailedSuites++
        continue
    }

    # Execute test binary
    & "$TestBin"
    if ($LASTEXITCODE -eq 0) {
        Write-Host "[PASS] Suite $SuiteName PASSED" -ForegroundColor Green
        $PassedSuites++
    } else {
        Write-Host "[FAIL] Suite $SuiteName FAILED (Exit Code: $LASTEXITCODE)" -ForegroundColor Red
        $FailedSuites++
    }
}

Write-Host "`n==========================================================" -ForegroundColor Cyan
Write-Host "  TEST EXECUTION SUMMARY" -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host "Suites Executed : $TotalSuites"
Write-Host "Suites Passed   : $PassedSuites" -ForegroundColor Green
Write-Host "Suites Failed   : $FailedSuites" -ForegroundColor $(if ($FailedSuites -eq 0) { "Green" } else { "Red" })

if ($FailedSuites -gt 0) {
    Write-Host "`n[FATAL] One or more unit test suites failed verification!" -ForegroundColor Red
    exit 1
}

Write-Host "`n[SUCCESS] 100% of firmware unit tests passed cleanly!" -ForegroundColor Green
exit 0
