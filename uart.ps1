$argsArray = 
    "/BAUD=460800", #Speed
    "/C=16" ,        #Serial port COM16
    "/DS"           #Disable displaying the "New connection" dialog on startup
Start-Process -FilePath "C:\Program Files (x86)\teraterm\ttermpro.exe" -ArgumentList $argsArray
