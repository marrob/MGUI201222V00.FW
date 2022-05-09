$argsArray = 
    "/BAUD=9600", #Speed
    "/C=21" ,        #Serial port COM16
    "/DS"           #Disable displaying the "New connection" dialog on startup
Start-Process -FilePath "C:\Program Files (x86)\teraterm\ttermpro.exe" -ArgumentList $argsArray
