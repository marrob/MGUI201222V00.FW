$argsArray = 
    "/BAUD=115200", #Speed
    "/C=4" ,        #Serial port COM5
    "/DS"           #Disable displaying the "New connection" dialog on startup
Start-Process -FilePath "C:\Program Files (x86)\teraterm\ttermpro.exe" -ArgumentList $argsArray
