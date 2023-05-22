rem netstat /r <-- use to see results of these

rem delete the default 0.0.0.0 route
route DELETE 0.0.0.0 192.168.1.1 

rem route all traffic through alex' (192.168.1.39) through an arbitrary gateway (0.0.0.0)
rem dont mask any traffic 0.0.0.0
route ADD 0.0.0.0 mask 0.0.0.0 192.168.1.39

rem set dns routing to use 8.8.8.8 (google's server)
netsh interface ip set dns "Wi-Fi" static 8.8.8.8
    
