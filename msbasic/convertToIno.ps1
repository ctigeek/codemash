$bytes = [System.IO.File]::ReadAllBytes("C:\Users\ctige\OneDrive\Documents\Code\msbasic\arduino.bin")

$basePath = "C:\Users\ctige\OneDrive\Documents\Arduino\6502computer"

Remove-Item -Path "$basePath\ROM.ino.old" -Force -ErrorAction SilentlyContinue
Rename-Item -Path "$basePath\ROM.ino" -NewName "$basePath\ROM.ino.old"
Start-Sleep -Seconds 1

$ROM_SIZE = 6144;  ## 0x1800 HEX, 6KB

$bank = 1;
$counter = 0
foreach ($val in $bytes) {
  if ($counter -eq 0) {
    if ($bank -gt 1) {
      " }`r`n" | Out-File "$basePath\ROM.ino" -Encoding ascii -Append
    }
    "`r`n void FillRom$($bank)(byte * buffer) { " | Out-File "$basePath\ROM.ino" -Encoding ascii -Append
  }
  "    buffer[$counter] = $([string]::Format("0x{0:x2};", [int]$val))" | Out-File "$basePath\ROM.ino" -Encoding ascii -Append
  $counter = $counter + 1
  if ($counter -eq $ROM_SIZE) {
    $counter = 0;
    $bank = $bank + 1;
  }
}

if ($counter -gt 0) {
  "     for (int i=$counter; i<ROM_SIZE; i++) { " | Out-File "$basePath\ROM.ino" -Encoding ascii -Append
  "        buffer[i] = 0x99;" | Out-File "$basePath\ROM.ino" -Encoding ascii -Append
  "     }" | Out-File "$basePath\ROM.ino" -Encoding ascii -Append
  $bank = $bank + 1;
}

" }" | Out-File "$basePath\ROM.ino" -Encoding ascii -Append

while ($bank -lt 7) {
  "`r`n void FillRom$($bank)(byte * buffer) {" | Out-File "$basePath\ROM.ino" -Encoding ascii -Append
  "     for (int i=0; i<ROM_SIZE; i++) {" | Out-File "$basePath\ROM.ino" -Encoding ascii -Append
  "        buffer[i] = 0x99;" | Out-File "$basePath\ROM.ino" -Encoding ascii -Append
  "     }" | Out-File "$basePath\ROM.ino" -Encoding ascii -Append
  " }" | Out-File "$basePath\ROM.ino" -Encoding ascii -Append
  $bank = $bank + 1;
}
