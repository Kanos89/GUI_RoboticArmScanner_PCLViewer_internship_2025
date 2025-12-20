<<<<<<< HEAD
MODULE MainModule

! Definition of variables for tcp ip connection
VAR socketdev serverSocket;
VAR socketdev clientSocket;
VAR string data;
VAR bool connected;
VAR bool ok; ! needed to use StrToVal, cf rapid technnical reference manual
VAR string position_str;
VAR num position_num;
VAR num nb_pos;

! Robot positions
CONST robtarget p1:=[[-1045.63,-1049.50,662.17],[0.57275,0.39155,0.41443,-0.58898],[-2,-1,0,1],[2919.98,9E+9,9E+9,9E+9,9E+9,9E+9]];
CONST robtarget p1:=[[-1045.63,-940.98,536.58],[0.57273,0.39155,0.41443,-0.58899],[-2,-1,0,1],[2919.98,9E+9,9E+9,9E+9,9E+9,9E+9]];
CONST robtarget p2:=[[-804.50,-940.98,536.56],[0.57272,0.39156,0.41444,-0.58899],[-2,-1,0,1],[2919.98,9E+9,9E+9,9E+9,9E+9,9E+9]];
CONST robtarget p3:=[[-811.89,-1105.92,536.55],[0.57272,0.39156,0.41446,-0.58898],[-2,-1,0,1],[2919.98,9E+9,9E+9,9E+9,9E+9,9E+9]];

! Configuration of real robot
TASK PERS tooldata Tooldata_2:=[TRUE,[[0,0,0],[1,0,0,0]],[1,[0,0,1],[1,0,0,0],0,0,0]];
TASK PERS wobjdata Workobject_2:=[FALSE,TRUE,"",[[1000,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

! Position array (robot targets)
VAR robtarget scan_positions{4};


PROC main()
        ! Initialize the array elements
        scan_positions{1} := p1;
        scan_positions{2} := p2;
        scan_positions{3} := p3;
        scan_positions{4} := p4;
        nb_pos := Dim(scan_positions, 1);
        
    WHILE TRUE DO
        ! Create and bind socket
        SocketCreate serverSocket;
        SocketBind serverSocket, "127.0.0.1", 1234;
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
            IF StrMatch(data, 1, "MOVE") = 1 THEN
                ! Extract position number from command
                ! Expected format: "MOVE 1", "MOVE 2", etc.
                position_str := StrPart(data, StrLen(data), 1);
                ok:= StrToVal(position_str,position_num) ;                    
                ! Validate position number
                IF position_num >= 1 AND position_num <= nb_pos THEN
                    MoveL scan_positions{position_num}, v200, fine, Tooldata_2\WObj:=Workobject_2;
                    ! Wait for the robot to reach and stop at the position
                    WaitRob \InPos;
                    ! Send confirmation back
                    SocketSend clientSocket \Str:="moved to position";
                ENDIF
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
=======
MODULE MainModule

! Definition of variables for tcp ip connection
VAR socketdev serverSocket;
VAR socketdev clientSocket;
VAR string data;
VAR bool connected;
VAR bool ok; ! needed to use StrToVal, cf rapid technnical reference manual
VAR string position_str;
VAR num position_num;
VAR num nb_pos;

! Robot positions
CONST robtarget p1:=[[-1045.63,-1049.50,662.17],[0.57275,0.39155,0.41443,-0.58898],[-2,-1,0,1],[2919.98,9E+9,9E+9,9E+9,9E+9,9E+9]];
CONST robtarget p1:=[[-1045.63,-940.98,536.58],[0.57273,0.39155,0.41443,-0.58899],[-2,-1,0,1],[2919.98,9E+9,9E+9,9E+9,9E+9,9E+9]];
CONST robtarget p2:=[[-804.50,-940.98,536.56],[0.57272,0.39156,0.41444,-0.58899],[-2,-1,0,1],[2919.98,9E+9,9E+9,9E+9,9E+9,9E+9]];
CONST robtarget p3:=[[-811.89,-1105.92,536.55],[0.57272,0.39156,0.41446,-0.58898],[-2,-1,0,1],[2919.98,9E+9,9E+9,9E+9,9E+9,9E+9]];

! Configuration of real robot
TASK PERS tooldata Tooldata_2:=[TRUE,[[0,0,0],[1,0,0,0]],[1,[0,0,1],[1,0,0,0],0,0,0]];
TASK PERS wobjdata Workobject_2:=[FALSE,TRUE,"",[[1000,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

! Position array (robot targets)
VAR robtarget scan_positions{4};


PROC main()
        ! Initialize the array elements
        scan_positions{1} := p1;
        scan_positions{2} := p2;
        scan_positions{3} := p3;
        scan_positions{4} := p4;
        nb_pos := Dim(scan_positions, 1);
        
    WHILE TRUE DO
        ! Create and bind socket
        SocketCreate serverSocket;
        SocketBind serverSocket, "127.0.0.1", 1234;
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
            IF StrMatch(data, 1, "MOVE") = 1 THEN
                ! Extract position number from command
                ! Expected format: "MOVE 1", "MOVE 2", etc.
                position_str := StrPart(data, StrLen(data), 1);
                ok:= StrToVal(position_str,position_num) ;                    
                ! Validate position number
                IF position_num >= 1 AND position_num <= nb_pos THEN
                    MoveL scan_positions{position_num}, v200, fine, Tooldata_2\WObj:=Workobject_2;
                    ! Wait for the robot to reach and stop at the position
                    WaitRob \InPos;
                    ! Send confirmation back
                    SocketSend clientSocket \Str:="moved to position";
                ENDIF
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
>>>>>>> 1f5935a291bc6e80918cf3a2c91adbc8e9a8f452
ENDMODULE