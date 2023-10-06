kawasaki robot controller 

Configuring the Robot Controller
The code example shows a controller that is connected to an In-Sight vision system with an IP address of 192.168.0.12. To find the IP address of the vision system from the In-Sight Explorer, right-click the vision system name in the In-Sight Network tree, and then select the IP address of the vision system.Select Properties.

 

.PROGRAM cognex()

;POINT FRAME1 is the part frame, where frame1 is calculated using

;the FRAME command: POINT FRAME1=FRAME(f.origin,f.x1,f.xy,f.origin)

;The coordinates are assumed to be received from the vision system in the part frame

$termchk=$CHR(13)+$CHR(10)

.ret1=-1

.ret1a=-1

.ret1a1=-1

.ret1b=-1

.ret1b1=-1

.ret1bc=-1

.ret2=-1

CALL tcp_open(10000); sock_id1 = CAMERA

IF sock_id1<0 THEN

RETURN

END

$receive=""

CALL tcp_recv(.ret1a,$receive)

IF .ret1a<0 GOTO quit

.$str = "admin"+$CHR(13)+$CHR(10)

CALL tcp_send(.ret1a1,.$str)

IF .ret1a1<0 GOTO quit

$receive = ""

CALL tcp_recv(.ret1b,$receive)

IF .ret1b<0 GOTO quit

$receive = ""

CALL tcp_recv(.ret1c,$receive)

IF .ret1c<0 GOTO quit

.$str = "GVA002"+$CHR(13)+$CHR(10)

CALL tcp_send(.ret1,.$str)

IF .ret1<0 GOTO quit

$receive = ""

CALL tcp_recv(.ret2,$receive)

IF .ret2<0 GOTO quit

$response = $receive

$ok = $DECODE($response,$termchk,0)

ok = VAL($ok)

IF ok <>1 GOTO quit

$response = $RIGHT($response,LEN($response)-4)

$response = $LEFT($response,LEN($response)-5)

$x = $DECODE($response,",",0)

x = VAL($x)

$temp = $DECODE($response,",",1)

$y = $DECODE($response,",",0)

y = VAL($y)

$temp = $DECODE($response,",",1)

o = VAL($response)

PRINT x,y,o

POINT pick = frame1 + TRANS(x,y,o)

HOME

LAPPRO pick, 100

LMOVE pick

CLAMP 1

LDEPART 100

HOME

quit:

CALL tcp_close

.END

.PROGAM tcp_close()

ret = 0

TCP_CLOSE ret,sock_id1

IF ret<0 THEN

PRINT "TCP_CLOSE error id = ",sock_id1

ELSE

PRINT "TCP_CLOSE OK id = ",sock_id1

END

.END

.PROGRAM tcp_open(.port)

tout_open = 60

ip[1] = 192

ip[2] = 168

ip[3] = 0

ip[4] = 12

port = .port

.er_count = 0

connect:

TIMER (2) = 0

TCP_CONNECT sock_id1,port,ip[1],tout_open

IF sock_id1<0 THEN

IF .er_count>=5 THEN

PRINT "Client Communication with Cognex has failed"

GOTO forgetit

ELSE

.er_count = .er_count+1

PRINT "TCP_CONNECT error id = ",sock_id1," error count = ",.er_count

GOTO connect

END

ELSE

PRINT "TCP_CONNECT OK id = ",sock_id1," with timep = ",TIMER(2)

END

forgetit:

RETURN

.END

.PROGRAM tcp_recv(.ret,$receive)

.ret = 0

.eret = -99

tout_rec = 60

max_length = 255

.$receive = ""

.num = 0

TCP_RECV .eret,sock_id1,.#recv_buff[1],.num,tout_rec,max_length

TIMER(3) = 0

IF .eret<0 THEN

PRINT "TCP_RECV error in RECV",.eret

PRINT ".num = ",.num

.ret = -1

ELSE

IF .num>0 THEN

IF .num*max_length<=255 THEN

PRINT "TCP_RECV OK in RECV",.eret

PRINT "Number of array elements = ",.num

FOR .j = 1 TO .num

.$receive = .$receive + .#recv_buf[.j]

PRINT "len(.$recv_buf[",.j,"] = ",LEN(.$recv_buf[.j])

PRINT ".$receive = ",.$receive

END

ELSE

.ret = -1

PRINT "String too long"

PRINT .$recv_buf[1]

END

ELSE

PRINT "Invalid response"

.ret = -1

END

.END

.PROGRAM tcp_send(.ret,.$data)

.$send_buf[1] = .$data

.buf_n = 1

.ret = 1

tout = 60

TCP_SEND .sret,sock_id1,.$send_buf[1],.buf_n,tout

IF .sret<0 THEN

.ret = -1

PRINT "TCP SEND error in SEND",.sret

ELSE

PRINT "TCP_SEND OK in SEND",.sret

PRINT "Sent string = ",$send_buf[1]

END

.END