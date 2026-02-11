MODULE EGM_IRB1300_C30
    ! Exemple RAPID pour IRB 1300 + contrôleur C30 (EGM en mode pose).
    ! Communication Device configuré dans RobotStudio:
    !   Name        : EGM_1
    !   Type        : UDPUC
    !   Remote Addr : 127.0.0.1
    !   Remote Port : 6511
    !   Local Port  : 0

    PERS tooldata tGripper := [TRUE,[[0,0,120],[1,0,0,0]],[0.5,[0,0,60],[1,0,0,0],0,0,0]];
    PERS wobjdata wobjEGM := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

    VAR egmident egmID;

    ! Fenêtres de correction (adapter selon votre process/sécurité)
    CONST egm_minmax mmPosX := [-2,2];
    CONST egm_minmax mmPosY := [-2,2];
    CONST egm_minmax mmPosZ := [-2,2];
    CONST egm_minmax mmRotX := [-1,1];
    CONST egm_minmax mmRotY := [-1,1];
    CONST egm_minmax mmRotZ := [-1,1];

    PROC main()
        TPWrite "EGM example IRB1300/C30 - start";

        MoveAbsJ [[0,0,0,0,90,0],[9E9,9E9,9E9,9E9,9E9,9E9]], v100, z50, tGripper;

        StartEgmPose;
        RunEgmPose 60;
        StopEgm;

        TPWrite "EGM example done";
    ENDPROC

    PROC StartEgmPose()
        ! Alloue un ID EGM
        EGMGetId egmID;

        ! Lie l'ID EGM au device UDPUC "EGM_1" configuré dans Communication
        ! Le profile "default" doit exister dans Motion -> External Motion Interface Data.
        EGMSetupUC ROB_1, egmID, "default", "EGM_1"\Pose;

        ! Active le mode EGM cartésien (pose TCP)
        EGMActPose egmID,
                   \Tool:=tGripper,
                   \WObj:=wobjEGM,
                   \x:=mmPosX,\y:=mmPosY,\z:=mmPosZ,
                   \Rx:=mmRotX,\Ry:=mmRotY,\Rz:=mmRotZ;

        TPWrite "EGM mode pose active";
    ENDPROC

    PROC RunEgmPose(num durationSec)
        VAR clock c1;
        ClkReset c1;
        ClkStart c1;

        ! Maintient EGM actif pendant durationSec secondes.
        WHILE ClkRead(c1) < durationSec DO
            ! EGMRunPose applique en continu les corrections reçues du PC.
            EGMRunPose egmID, EGM_STOP_HOLD\NoWaitCond;
            WaitTime 0.004;  ! ~250 Hz loop RAPID (adapter selon charge CPU)
        ENDWHILE

        ClkStop c1;
    ENDPROC

    PROC StopEgm()
        EGMReset egmID;
        TPWrite "EGM reset";
    ENDPROC

ENDMODULE
