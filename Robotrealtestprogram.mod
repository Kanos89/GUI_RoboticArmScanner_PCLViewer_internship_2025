MODULE T_ROB_Nathan
VAR socketdev serverSocket;
VAR socketdev clientSocket;
VAR string data;
VAR bool connected;
CONST robtarget initp:=[[-1045.63,-1049.50,662.17],[0.57275,0.39155,0.41443,-0.58898],[-2,-1,0,1],[2919.98,9E+9,9E+9,9E+9,9E+9,9E+9]];
CONST robtarget p1:=[[-1045.63,-940.98,536.58],[0.57273,0.39155,0.41443,-0.58899],[-2,-1,0,1],[2919.98,9E+9,9E+9,9E+9,9E+9,9E+9]];
CONST robtarget p2:=[[-804.50,-940.98,536.56],[0.57272,0.39156,0.41444,-0.58899],[-2,-1,0,1],[2919.98,9E+9,9E+9,9E+9,9E+9,9E+9]];
CONST robtarget p3:=[[-811.89,-1105.92,536.55],[0.57272,0.39156,0.41446,-0.58898],[-2,-1,0,1],[2919.98,9E+9,9E+9,9E+9,9E+9,9E+9]];

TASK PERS tooldata Tooldata_2:=[TRUE,[[0,0,0],[1,0,0,0]],[1,[0,0,1],[1,0,0,0],0,0,0]];
TASK PERS wobjdata Workobject_2:=[FALSE,TRUE,"",[[1000,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

PROC testing()
    WHILE TRUE DO
        ! Create and bind socket
        SocketCreate serverSocket;
        SocketBind serverSocket, "192.168.125.1", 1234;
        SocketListen serverSocket;

        TPWrite "Waiting for connection...";
        SocketAccept serverSocket, clientSocket;
        TPWrite "Client connected.";
        connected := TRUE ;
        
        WHILE connected DO
            ! Receive command from Python client
            SocketReceive clientSocket \Str:=data;
            TPWrite "Received: " + data;

            ! Execute based on command
            IF data = "MOVE Initial Position" THEN
                MoveL initp, v200, fine, Tooldata_2\WObj := Workobject_2;
                ! Wait for the robot to reach and stop at the position
                WaitRob \InPos;
                ! Send confirmation back
                SocketSend clientSocket \Str:="moved to position";
            ELSEIF data = "MOVE Scan Position 1" THEN
                MoveL p1, v200, fine, Tooldata_2\WObj := Workobject_2;
                ! Wait for the robot to reach and stop at the position
                WaitRob \InPos;
                ! Send confirmation back
                SocketSend clientSocket \Str:="moved to position";
            ELSEIF data = "MOVE Scan Position 2" THEN
                MoveL p2, v200, fine, Tooldata_2\WObj := Workobject_2;
                ! Wait for the robot to reach and stop at the position
                WaitRob \InPos;
                ! Send confirmation back
                SocketSend clientSocket \Str:="moved to position";
            ELSEIF data = "MOVE Scan Position 3" THEN
                MoveL p3, v200, fine, Tooldata_2\WObj := Workobject_2;
                ! Wait for the robot to reach and stop at the position
                WaitRob \InPos;
                ! Send confirmation back
                SocketSend clientSocket \Str:="moved to position";
            ELSEIF data = "DISCONNECT" THEN
                ! Close connection
                ! Send disconnect message
                SocketSend clientSocket \Str:="OK";
                SocketClose clientSocket;
                SocketClose serverSocket;
                connected := FALSE;
            ELSE
                TPWrite "Unknown command.";
            ENDIF

        
        ENDWHILE
    ENDWHILE
ENDPROC
ENDMODULE