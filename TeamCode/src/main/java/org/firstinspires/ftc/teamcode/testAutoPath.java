package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Rect;
import org.opencv.core.Size;

//dab
@Autonomous(name = "testAutoPath", group = "Prototyping")

public class testAutoPath extends LinearOpMode {
    Servo rightLock;// = hardwareMap.servo.get("rightLock");
    Servo leftLock;// = hardwareMap.servo.get("leftLock");
    DcMotor leftWheel1;// = hardwareMap.dcMotor.get("leftWheel");
    DcMotor leftWheel2;// = hardwareMap.dcMotor.get("leftWheel2");
    DcMotor rightWheel1;// = hardwareMap.dcMotor.get("rightWheel");
    DcMotor rightWheel2;// = hardwareMap.dcMotor.get("rightWheel2");
    DcMotor leftArm;// = hardwareMap.dcMotor.get("leftArm");
    DcMotor rightArm;// = hardwareMap.dcMotor.get("rightArm");
    DcMotor armExtendRight;// = hardwareMap.dcMotor.get("armExtend");
    DcMotor armExtendLeft;// = hardwareMap.dcMotor.get("armExtend2");

//    private GoldDetector detector;
//    double alignSize=70;
//    double alignX    = 270;//  (640 / 2) +0; // Center point in X Pixels
//    double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
//    double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels
//    double scale = (640)/2;//-alignSize

    private ElapsedTime runtime = new ElapsedTime();
    private final double TICKS_PER_WHEEL_ROTATION = 1120;

    int subStep=0;
    private int step;

    private enum state {TURN_TO_WALL, MOVE_TO_WALL, MOVE_AWAY_FROM_WALL, TURN_TO_DEPOT, MOVE_TO_DEPOT}
    double stage = -4;

    int data[][]={{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,1},{0,2},{0,2},{-1,4},{-2,5},{-3,6},{-5,9},{-6,11},{-9,15},{-12,16},{-17,20},{-24,26},{-31,34},{-38,41},{-46,50},{-56,63},{-65,71},{-78,84},{-91,94},{-103,106},{-117,118},{-130,131},{-145,144},{-159,157},{-173,168},{-186,181},{-202,194},{-216,209},{-231,218},{-245,232},{-259,245},{-276,259},{-290,272},{-304,284},{-319,296},{-333,308},{-349,323},{-361,334},{-374,346},{-387,356},{-399,369},{-414,380},{-429,395},{-444,406},{-457,418},{-473,430},{-486,444},{-501,454},{-516,469},{-529,478},{-543,492},{-557,503},{-572,519},{-586,528},{-599,542},{-613,556},{-627,568},{-638,581},{-648,593},{-657,606},{-666,616},{-673,626},{-679,641},{-684,648},{-689,663},{-692,672},{-694,683},{-695,691},{-698,700},{-701,709},{-704,713},{-707,720},{-709,728},{-712,736},{-716,744},{-720,755},{-724,764},{-728,776},{-733,787},{-738,801},{-743,812},{-748,827},{-756,838},{-763,852},{-770,862},{-780,875},{-789,886},{-798,901},{-809,913},{-819,929},{-829,940},{-839,952},{-851,966},{-862,978},{-876,995},{-890,1008},{-903,1025},{-918,1040},{-929,1051},{-939,1063},{-948,1072},{-957,1082},{-965,1093},{-973,1097},{-980,1102},{-985,1108},{-990,1112},{-995,1117},{-1000,1120},{-1006,1124},{-1011,1128},{-1019,1136},{-1030,1145},{-1039,1150},{-1047,1158},{-1058,1169},{-1071,1177},{-1086,1189},{-1100,1198},{-1116,1210},{-1131,1218},{-1148,1231},{-1163,1241},{-1179,1253},{-1194,1265},{-1209,1274},{-1224,1287},{-1239,1294},{-1254,1306},{-1269,1316},{-1287,1326},{-1302,1337},{-1316,1342},{-1331,1349},{-1343,1356},{-1356,1360},{-1369,1365},{-1381,1371},{-1396,1376},{-1409,1381},{-1422,1384},{-1435,1388},{-1448,1393},{-1461,1399},{-1477,1405},{-1491,1410},{-1507,1418},{-1521,1426},{-1539,1432},{-1557,1442},{-1575,1451},{-1594,1458},{-1610,1466},{-1625,1473},{-1640,1481},{-1657,1485},{-1675,1493},{-1691,1501},{-1709,1506},{-1727,1514},{-1748,1523},{-1768,1529},{-1791,1536},{-1811,1544},{-1831,1553},{-1851,1557},{-1870,1564},{-1887,1572},{-1905,1577},{-1916,1582},{-1926,1588},{-1932,1593},{-1935,1597},{-1935,1597},{-1936,1599},{-1937,1599},{-1941,1599},{-1944,1601},{-1952,1600},{-1959,1605},{-1968,1610},{-1979,1616},{-1989,1625},{-1999,1630},{-2008,1638},{-2022,1650},{-2034,1660},{-2047,1672},{-2059,1678},{-2071,1688},{-2082,1696},{-2093,1704},{-2102,1712},{-2111,1721},{-2120,1724},{-2130,1733},{-2140,1742},{-2151,1748},{-2165,1756},{-2179,1764},{-2193,1769},{-2209,1776},{-2228,1787},{-2247,1796},{-2266,1806},{-2281,1817},{-2297,1822},{-2312,1831},{-2325,1841},{-2337,1845},{-2345,1851},{-2351,1856},{-2354,1858},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1859},{-2355,1857},{-2355,1855},{-2354,1852},{-2352,1849},{-2352,1846},{-2350,1841},{-2348,1841},{-2344,1833},{-2339,1826},{-2334,1816},{-2329,1810},{-2324,1800},{-2319,1791},{-2313,1778},{-2307,1768},{-2301,1756},{-2296,1744},{-2289,1731},{-2284,1720},{-2279,1711},{-2275,1700},{-2271,1693},{-2267,1680},{-2263,1673},{-2258,1661},{-2253,1650},{-2249,1643},{-2243,1630},{-2238,1621},{-2231,1606},{-2222,1597},{-2215,1584},{-2206,1575},{-2196,1558},{-2184,1546},{-2171,1528},{-2159,1514},{-2147,1498},{-2135,1480},{-2127,1466},{-2117,1449},{-2108,1433},{-2100,1414},{-2092,1397},{-2086,1381},{-2080,1361},{-2073,1341},{-2066,1322},{-2056,1300},{-2046,1278},{-2033,1261},{-2020,1241},{-2008,1221},{-1993,1205},{-1978,1189},{-1961,1170},{-1942,1158},{-1927,1144},{-1909,1132},{-1892,1121},{-1873,1104},{-1851,1093},{-1831,1078},{-1811,1069},{-1793,1058},{-1778,1048},{-1761,1037},{-1745,1025},{-1728,1015},{-1711,1002},{-1694,993},{-1678,982},{-1661,971},{-1641,957},{-1621,944},{-1602,929},{-1583,918},{-1564,904},{-1545,893},{-1527,880},{-1512,871},{-1492,858},{-1474,847},{-1459,836},{-1446,829},{-1430,816},{-1415,809},{-1401,797},{-1386,786},{-1373,779},{-1358,766},{-1345,760},{-1332,746},{-1319,737},{-1304,724},{-1289,713},{-1275,700},{-1260,688},{-1245,677},{-1231,665},{-1215,654},{-1200,641},{-1183,629},{-1167,616},{-1152,606},{-1136,592},{-1118,578},{-1101,569},{-1087,554},{-1071,543},{-1056,530},{-1042,521},{-1028,508},{-1014,496},{-999,485},{-983,472},{-970,462},{-956,449},{-943,439},{-928,427},{-911,416},{-894,401},{-874,386},{-856,373},{-842,358},{-828,349},{-816,336},{-802,325},{-789,312},{-778,303},{-764,290},{-753,280},{-740,270},{-731,260},{-722,253},{-712,243},{-704,234},{-696,227},{-687,218},{-679,209},{-670,203},{-662,194},{-652,184},{-645,179},{-638,172},{-631,166},{-624,160},{-617,159},{-613,153},{-608,149},{-604,145},{-599,140},{-591,137},{-584,128},{-574,118},{-562,109},{-550,96},{-540,89},{-530,78},{-519,67},{-507,59},{-497,48},{-484,41},{-475,29},{-464,18},{-455,13},{-445,2},{-434,-8},{-423,-19},{-410,-32},{-398,-41},{-385,-55},{-372,-64},{-360,-78},{-345,-90},{-330,-104},{-317,-115},{-304,-128},{-290,-139},{-279,-150},{-267,-158},{-255,-168},{-241,-177},{-229,-188},{-219,-198},{-208,-203},{-197,-214},{-186,-222},{-175,-227},{-166,-236},{-156,-244},{-146,-249},{-134,-262},{-122,-272},{-112,-279},{-101,-290},{-90,-296},{-80,-307},{-67,-319},{-57,-327},{-44,-338},{-33,-345},{-22,-356},{-10,-367},{1,-375},{13,-386},{25,-392},{37,-406},{51,-415},{65,-428},{77,-440},{84,-447},{91,-458},{99,-465},{105,-476},{111,-486},{117,-490},{121,-498},{125,-504},{131,-510},{135,-511},{140,-515},{144,-518},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{146,-519},{145,-519},{145,-519},{145,-519},{145,-519},{145,-519},{144,-519},{144,-519},{144,-519},{144,-519},{144,-519},{144,-519},{144,-519},{144,-519},{144,-519},{144,-519},{144,-519},{144,-519},{144,-519},{144,-519},{142,-519},{142,-519},{141,-519},{140,-519},{139,-519},{138,-518},{137,-517},{136,-516},{135,-516},{135,-516},{135,-516},{135,-516},{135,-516},{134,-516},{134,-516},{134,-516},{133,-516},{133,-516},{131,-516},{129,-515},{128,-513},{125,-511},{122,-511},{120,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512},{117,-512}};
    int pos = 0;
    int setL = 0;
    int setR = 0;
    int errorR;
    int errorL;

    double p = .01;
    double i = .00001;
    double d = .00001;

    boolean aligned = false;

    public void runOpMode() throws InterruptedException {
        rightLock = hardwareMap.servo.get("rightLock");
        leftLock = hardwareMap.servo.get("leftLock");
        leftWheel1 = hardwareMap.dcMotor.get("leftWheel");
        leftWheel2 = hardwareMap.dcMotor.get("leftWheel2");
        rightWheel1 = hardwareMap.dcMotor.get("rightWheel");
        rightWheel2 = hardwareMap.dcMotor.get("rightWheel2");
        leftArm = hardwareMap.dcMotor.get("leftArm");
        rightArm = hardwareMap.dcMotor.get("rightArm");
        armExtendRight = hardwareMap.dcMotor.get("armExtend");
        armExtendLeft = hardwareMap.dcMotor.get("armExtend2");

        //rightWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        step = -1;

//        telemetry.addData("Status", "DogeCV 2018.0 - Gold Detector test");
//        // Setup detector
//        detector = new GoldDetector(); // Create detector
//        detector.setAdjustedSize(new Size(480, 270)); // Set detector size
//        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize detector with app context and camera
//        detector.useDefaults(); // Set default detector settings
//        // Optional tuning
//        detector.downscale = 0.4; // How much to downscale the input frames
//
//        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
//        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
//        detector.maxAreaScorer.weight = 0.005;
//
//        detector.ratioScorer.weight = 5;
//        detector.ratioScorer.perfectRatio = 1.0;
//        detector.enable(); // Start detector


        //prepares and resets robot by resetting encoders
        //resetDriveEncoders();

        //prepares robot for arm movement by setting arm motors to go to a certain position-modifiable by user input (the d-pad)
//        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //HSV value array

        //Sends color sensor input values to the phone
        waitForStart();


        rightWheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            telemetry.update();


           // telemetry.addLine()
               //     .addData("step", step);
            telemetry.addLine()
                    .addData("rightWheel1: ", leftWheel1.getCurrentPosition())
                    .addData("rightWheel1: ", rightWheel1.getCurrentPosition());
            //.addData("TICKS_PER_WHEEL_ROTATION: ", TICKS_PER_WHEEL_ROTATION);
          //  telemetry.addData("arm::::::", leftArm.getCurrentPosition());
            //telemetry.addData("armExtend", armExtendLeft.getCurrentPosition());
            //telemetry.addData("armExtend2", armExtendRight.getCurrentPosition());

            if (step == -1) {
                if (runtime.seconds() > .025) {
                    setL = data[pos][0];
                    setR = data[pos][1];
                    pos++;
                    runtime.reset();
                }
                errorR= setR-rightWheel1.getCurrentPosition();
                errorL = setL-leftWheel1.getCurrentPosition();
                double pr;
                double pl;

                pr= errorR*p;
                pl=errorL*p;

                rightWheel1.setPower(pr);
                rightWheel2.setPower(pr);
                leftWheel1.setPower(pl);
                leftWheel2.setPower(pl);

                telemetry.addData("pr",pr);
                telemetry.addData("pl",pl);
                telemetry.addData("errorL:", errorL);
                telemetry.addData("errorR",errorR);



            }
//            if (step == 0) {
//                leftLock.setPosition(.25);
//                rightLock.setPosition(.4);
//                if(leftArm.getCurrentPosition()>-4250) {
//                    //leftArm.setPower(.3);
//                    rightArm.setPower(-.3);
//
//                }
//                else if (leftArm.getCurrentPosition()>-5000){
//                    //leftArm.setPower(.65);
//                    rightArm.setPower(-.65);
//                    if(rightWheel1.getCurrentPosition()>-400) {
//                        leftWheel2.setPower(-.4);
//                        leftWheel1.setPower(-.4);
//                        rightWheel2.setPower(-.4);
//                        rightWheel1.setPower(-.4);
//                    }
//                    else{
//                        leftWheel2.setPower(0);
//                        leftWheel1.setPower(0);
//                        rightWheel2.setPower(0);
//                        rightWheel1.setPower(0);
//                    }
//                }
//                else{
//                    leftWheel2.setPower(0);
//                    leftWheel1.setPower(0);
//                    rightWheel2.setPower(0);
//                    rightWheel1.setPower(0);
//                   // leftArm.setPower(0);
//                    rightArm.setPower(0);
//                    step=1;
//                    runtime.reset();
//                }
//
//            }
//            if(step==1){
//                //leftArm.setPower(-.55);
//                 rightArm.setPower(.55);
//                if(runtime.seconds()>2.5){
//                    leftArm.setPower(0);
//                    rightArm.setPower(0);
//                    runtime.reset();
//                    step=2;
//                    rightWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    leftWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    rightWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    leftWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                }
//            }
//            if(step==2) {
//                Rect bestRect = detector.getFoundRect();
//
//                double xPos = bestRect.x + (bestRect.width / 2);
//
//                if (xPos < alignXMax && xPos > alignXMin) {
//                    aligned = true;
//                } else {
//                    aligned = false;
//                }
//                telemetry.addData("aligned ", aligned);
//
//                telemetry.addData("xpos ", xPos);
//                telemetry.addData("amax ", alignXMax);
//                telemetry.addData("amin ", alignXMin);
//                if(!(xPos>0)){
//                    if(runtime.seconds()<4) {
//                        rightWheel1.setPower(0);
//                        rightWheel2.setPower(0);
//                        leftWheel1.setPower(0);
//                        leftWheel2.setPower(0);
//                    }
//                    telemetry.addLine("not detected");
//
//                    if(runtime.seconds()>4 && runtime.seconds()<5){
//                        if(rightWheel1.getCurrentPosition()>-150) {
//                            rightWheel1.setPower(-.4);
//                            rightWheel2.setPower(-.4);
//                            leftWheel1.setPower(.4);
//                            leftWheel2.setPower(.4);
//                        }
//                        else{
//                            rightWheel1.setPower(0);
//                            rightWheel2.setPower(0);
//                            leftWheel1.setPower(0);
//                            leftWheel2.setPower(0);
//                        }
//                    }
//                    if(runtime.seconds()>5 && runtime.seconds()<7){
//                        if(rightWheel1.getCurrentPosition()<150) {
//                            rightWheel1.setPower(.4);
//                            rightWheel2.setPower(.4);
//                            leftWheel1.setPower(-.4);
//                            leftWheel2.setPower(-.4);
//                        }
//                        else{
//                            rightWheel1.setPower(0);
//                            rightWheel2.setPower(0);
//                            leftWheel1.setPower(0);
//                            leftWheel2.setPower(0);
//                        }
//                    }
//                    if(runtime.seconds()>7){
//                        if(rightWheel1.getCurrentPosition()>1) {
//                            rightWheel1.setPower(-.4);
//                            rightWheel2.setPower(-.4);
//                            leftWheel1.setPower(.4);
//                            leftWheel2.setPower(.4);
//                        }
//                        else{
//                            runtime.reset();
//                            step++;
//                            rightWheel1.setPower(0);
//                            rightWheel2.setPower(0);
//                            leftWheel1.setPower(0);
//                            leftWheel2.setPower(0);
//                        }
//
//                    }
//                }
//                else if (xPos > alignXMax) {
//                    double power = ((xPos - alignXMax) / scale) * .12 + .11;
//                    rightWheel1.setPower(-power);
//                    rightWheel2.setPower(-power);
//                    leftWheel1.setPower(power);
//                    leftWheel2.setPower(power);
//                    telemetry.addData("powL: ", power);
//                    telemetry.addLine("turning left");
//                    runtime.reset();
//                } else if (xPos < alignXMin) {
//                    double power = ((alignXMin - xPos) / scale) * .12 + .11 ;
//                    rightWheel1.setPower(power);
//                    rightWheel2.setPower(power);
//                    leftWheel1.setPower(-power);
//                    leftWheel2.setPower(-power);
//                    telemetry.addData("powR: ", power);
//                    telemetry.addLine("turning right");
//                    runtime.reset();
//                } else {
//                    rightWheel1.setPower(0);
//                    rightWheel2.setPower(0);
//                    leftWheel1.setPower(0);
//                    leftWheel2.setPower(0);
//                    telemetry.addLine("found");
//                    telemetry.addData("secks: ", runtime.seconds());
//                    if(runtime.seconds()>.3){
//                        runtime.reset();
//                        step++;
//                        //resetDriveEncoders();
//                    }
//                }
//
//
//            }
//            else if (step==3){
//                if(rightWheel1.getCurrentPosition()<1500) {
//                    leftWheel2.setPower(.6);
//                    leftWheel1.setPower(.6);
//                    rightWheel2.setPower(.6);
//                    rightWheel1.setPower(.6);
//                }
//                else {
//                    rightWheel1.setPower(0);
//                    rightWheel2.setPower(0);
//                    leftWheel1.setPower(0);
//                    leftWheel2.setPower(0);
//                }
//                //stage = 2;
//            }
//
//    /*
////sets the step of where we are in the process to one
//            // step = 1;
////enables the if statement for lander_unlatched
//            lander_unlatched = true;
////moves the robot from the lander to where the samples are
//            if (step == 1) {
////                leftWheel.setPower(0.7);
////                rightWheel.setPower(0.5);
////                leftWheel.setTargetPosition(1621);
////                rightWheel.setTargetPosition(820);
//                step = 2;
//            }
//            if (step == 2) {
////                if (hsvValues[0] > 50) {
////                    leftWheel.setPower(-0.7);
////                    rightWheel.setPower(-0.5);
////                    leftWheel.setTargetPosition(1221);
//                step = 3;
//            }
////            if step 2 is finished, makes the robot turn toward the wall, then drives against the wall.
//            if (step == 3) {
//                moveToDepot();
//            }
//            //1150 Target Units == about 1 foot, 96 Units/inch
////sets the wheel power to 0 to limit movement
////                if (rightWheel.getCurrentPosition() == -rotation) {
//            // rightWheel.setPower(0);
//            //leftWheel.setPower(0);
////                }
////               if (leftWheel.getCurrentPosition() == 100) {
////                    leftWheel.setPower(0);
////                    rightWheel.setPower(0);
////                }
//
//            sample_ready = true;
//
////        if (sample_ready) {
////
//        }
//
//        */
//        }
//    }
//
//    private void resetDriveEncoders() {
//        telemetry.addLine("RESET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n" +
//                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//        leftWheel1.setPower(0);
//        leftWheel2.setPower(0);
//        rightWheel1.setPower(0);
//        rightWheel2.setPower(0);
//        leftWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftWheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftWheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightWheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightWheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//    }
//
//    private void moveToDepot() throws InterruptedException {
//
//        if ((subStep * 2) % 2 == 1) {
//            resetDriveEncoders();
//            subStep += 0.5;
//        } else if (subStep == state.TURN_TO_WALL.ordinal()) {
//            // turns the robot 90 degrees counter clockwise
//            double target = .75;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
//                subStep += 0.5;
//            }
//        }
////
//        else if (subStep == state.MOVE_TO_WALL.ordinal()) {
//            // moves the robot forward up against the wall
//
//            double target = 3;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
//                subStep += 0.5;
//            }
//        } else if (subStep == state.MOVE_AWAY_FROM_WALL.ordinal()) {
//            // moves the robot backward away from the wall
//            double target = -1;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !leftWheel2.isBusy()) {
//                subStep += 0.5;
//
//            }
//        } else if (subStep == state.TURN_TO_DEPOT.ordinal()) {
//            // turns the robot counter-clockwise to line up with the depot
//            double target = 1;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
//                subStep += 0.5;
//            }
//        } else if (subStep == state.MOVE_TO_DEPOT.ordinal()) {
//            // moves the robot forward up to the depot
//            double target = 5;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
//                subStep += 0.5;
//            }
//        }
            //step = 4;
        }
    }
}
