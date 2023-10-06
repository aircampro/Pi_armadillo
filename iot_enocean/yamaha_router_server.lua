-- Create a TCP object and listen for a connection
-- if you get $send_temp$ then send the temperature from the internal hardware probe on tcp and to machinist iot
-- Yamaha Router Lua API

temperature_router = 0
tcp = rt.socket.tcp()
tcp:setoption("reuseaddr", true)
res, err = tcp:bind("192.168.100.1", 11111)
if not res and err then
 print(err)
 os.exit(1)
end
res, err = tcp:listen()
if not res and err then
 print(err)
 os.exit(1)
end

while 1 do

 control = assert(tcp:accept())
 raddr, rport = control:getpeername()
 print("remote host address:" .. raddr .. " port:" .. rport)
 -- Set receive wait timeout to 30 seconds
 control:settimeout(30)
 while 1 do
 
   msg, err, partial = control:receive()
   if (msg) then
     -- local data
     -- Process the received data (as an example, here it is outputting to the console)
	 -- we send the request string $send_temp$ 
     print(msg)
	 pattern="$send_temp$"
	 if string.match(msg, pattern) then
       -- Now Read the temperature 
       th, err = rt.hw.open("thermometer1")
       if (th) then
        temperature_router = th:read()
        th:close()
       end

       if (temperature_router > threshold) then
         led, str = rt.hw.open("status-led1")
         if (led) then
           led:blink(500, 500)
         end
       else
         if (led) then
           led:off()
           led:close()
         end
       end
       -- Send temperature as a response to incoming message
       sent, err = control:send(temperature_router)
       if (err) then
         print("send error(" .. err .. ")")
         control:close()
         break
       end
       -- send daya to machinist iot via http api
	   string.format("status=Tunnel Down: %s\n", string.match(array[1], pattern)))
       machinst_json = string.format("{
       \"agent\": \"GPIO\",
       \"metrics\": [
          {
            \"name\": \"router internal temperature\",
            \"namespace\": \"Yamaha Router Temp Sensor\",
            \"data_point\": {
            \"value\": %s
             }
          }
        ]
       }",temperature_router)
       req_t = {
       url = "https://gw.machinist.iij.jp/endpoint",
       method = "POST",
       content_type = "application/json",
       authorization = "Bearer YOR_API_KEY"
       }
       req_t.post_text = machinst_json
       rsp_t = rt.httprequest(req_t) 
	  end 
    else
       if (err ~= "closed") then
         print(os.date() .. "receive error(" .. err .. ")")
         if (partial) then
           print("receive partial data(" .. partial .. ")")
         end
       end
       control:close()
       break;
    end
   
  end
   
end
