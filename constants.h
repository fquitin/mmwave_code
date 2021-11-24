//
// Copyright ULB BEAMS-EE
// Author: François QUITIN
//



// Register values
extern const std::string REG1 = "00000000000143E0";
extern const std::string REG_TEMP = "000500000004dcd5";

// Responses
extern const std::string CHIP_OK = "AMO:4 chip setting complite ok";
extern const std::string AMO_OK = "AMO:ok";


// Convert angles to register values for AiP
std::string* angle_to_reg(std::string degrees, std::string direction)
{
    std::string* angle_reg_hex = new std::string[4];

    if (direction == "UP"){
	if (degrees == "DEG_0"){angle_reg_hex[0]="000820"; angle_reg_hex[1]="000820"; angle_reg_hex[2]="820000"; angle_reg_hex[3]="820000";}
        else if (degrees == "DEG_11_25"){angle_reg_hex[0]="080822"; angle_reg_hex[1]="080822"; angle_reg_hex[2]="926184"; angle_reg_hex[3]="926184";}
        else if (degrees == "DEG_22_25"){angle_reg_hex[0]="100824"; angle_reg_hex[1]="100824"; angle_reg_hex[2]="a2c308"; angle_reg_hex[3]="a2c308";}
        else if (degrees == "DEG_33_75"){angle_reg_hex[0]="180826"; angle_reg_hex[1]="180826"; angle_reg_hex[2]="b3248c"; angle_reg_hex[3]="b3248c";}
        else if (degrees == "DEG_45"){angle_reg_hex[0]="200828"; angle_reg_hex[1]="200828"; angle_reg_hex[2]="c38610"; angle_reg_hex[3]="c38610";}
        else if (degrees == "DEG_56_25"){angle_reg_hex[0]="28082a"; angle_reg_hex[1]="28082a"; angle_reg_hex[2]="d3e794"; angle_reg_hex[3]="d3e794";}
        else if (degrees == "DEG_67_5"){angle_reg_hex[0]="30082c"; angle_reg_hex[1]="30082c"; angle_reg_hex[2]="e04918"; angle_reg_hex[3]="e04918";}
        else if (degrees == "DEG_78_75"){angle_reg_hex[0]="38082e"; angle_reg_hex[1]="38082e"; angle_reg_hex[2]="f0aa9c"; angle_reg_hex[3]="f0aa9c";}
        else if (degrees == "DEG_90"){angle_reg_hex[0]="400830"; angle_reg_hex[1]="400830"; angle_reg_hex[2]="010c20"; angle_reg_hex[3]="010c20";}
        else if (degrees == "DEG_101_2"){angle_reg_hex[0]="480832"; angle_reg_hex[1]="480832"; angle_reg_hex[2]="116da4"; angle_reg_hex[3]="116da4";}
        else if (degrees == "DEG_112_5"){angle_reg_hex[0]="500834"; angle_reg_hex[1]="500834"; angle_reg_hex[2]="21cf28"; angle_reg_hex[3]="21cf28";}
        else if (degrees == "DEG_123_7"){angle_reg_hex[0]="580836"; angle_reg_hex[1]="580836"; angle_reg_hex[2]="3220ac"; angle_reg_hex[3]="3220ac";}
        else if (degrees == "DEG_135"){angle_reg_hex[0]="600838"; angle_reg_hex[1]="600838"; angle_reg_hex[2]="428230"; angle_reg_hex[3]="428230";}
        else if (degrees == "DEG_146_2"){angle_reg_hex[0]="68083a"; angle_reg_hex[1]="68083a"; angle_reg_hex[2]="52e3b4"; angle_reg_hex[3]="52e3b4";}
        else if (degrees == "DEG_157_5"){angle_reg_hex[0]="70083c"; angle_reg_hex[1]="70083c"; angle_reg_hex[2]="634538"; angle_reg_hex[3]="634538";}
        else if (degrees == "DEG_168_7"){angle_reg_hex[0]="78083e"; angle_reg_hex[1]="78083e"; angle_reg_hex[2]="73a6bc"; angle_reg_hex[3]="73a6bc";}
        else if (degrees == "DEG_180"){angle_reg_hex[0]="800800"; angle_reg_hex[1]="800800"; angle_reg_hex[2]="800800"; angle_reg_hex[3]="800800";}
    }
    else if (direction == "DOWN"){
	if (degrees == "DEG_0"){angle_reg_hex[0]="000820"; angle_reg_hex[1]="000820"; angle_reg_hex[2]="820000"; angle_reg_hex[3]="820000";}
        else if (degrees == "DEG_11_25"){angle_reg_hex[0]="1069a4"; angle_reg_hex[1]="1069a4"; angle_reg_hex[2]="8a0002"; angle_reg_hex[3]="8a0002";}
        else if (degrees == "DEG_22_25"){angle_reg_hex[0]="20cb28"; angle_reg_hex[1]="20cb28"; angle_reg_hex[2]="920004"; angle_reg_hex[3]="920004";}
        else if (degrees == "DEG_33_75"){angle_reg_hex[0]="312cac"; angle_reg_hex[1]="312cac"; angle_reg_hex[2]="9a0006"; angle_reg_hex[3]="9a0006";}
        else if (degrees == "DEG_45"){angle_reg_hex[0]="418e30"; angle_reg_hex[1]="418e30"; angle_reg_hex[2]="a20008"; angle_reg_hex[3]="a20008";}
        else if (degrees == "DEG_56_25"){angle_reg_hex[0]="51efb4"; angle_reg_hex[1]="51efb4"; angle_reg_hex[2]="aa000a"; angle_reg_hex[3]="aa000a";}
        else if (degrees == "DEG_67_5"){angle_reg_hex[0]="624138"; angle_reg_hex[1]="624138"; angle_reg_hex[2]="b2000c"; angle_reg_hex[3]="b2000c";}
        else if (degrees == "DEG_78_75"){angle_reg_hex[0]="72a2bc"; angle_reg_hex[1]="72a2bc"; angle_reg_hex[2]="ba000e"; angle_reg_hex[3]="ba000e";}
        else if (degrees == "DEG_90"){angle_reg_hex[0]="830400"; angle_reg_hex[1]="830400"; angle_reg_hex[2]="c20010"; angle_reg_hex[3]="c20010";}
        else if (degrees == "DEG_101_2"){angle_reg_hex[0]="936584"; angle_reg_hex[1]="936584"; angle_reg_hex[2]="ca0012"; angle_reg_hex[3]="ca0012";}
        else if (degrees == "DEG_112_5"){angle_reg_hex[0]="a3c708"; angle_reg_hex[1]="a3c708"; angle_reg_hex[2]="d20014"; angle_reg_hex[3]="d20014";}
        else if (degrees == "DEG_123_7"){angle_reg_hex[0]="b0288c"; angle_reg_hex[1]="b0288c"; angle_reg_hex[2]="da0016"; angle_reg_hex[3]="da0016";}
        else if (degrees == "DEG_135"){angle_reg_hex[0]="c08a10"; angle_reg_hex[1]="c08a10"; angle_reg_hex[2]="e20018"; angle_reg_hex[3]="e20018";}
        else if (degrees == "DEG_146_2"){angle_reg_hex[0]="d0eb94"; angle_reg_hex[1]="d0eb94"; angle_reg_hex[2]="ea001a"; angle_reg_hex[3]="ea001a";}
        else if (degrees == "DEG_157_5"){angle_reg_hex[0]="e14d18"; angle_reg_hex[1]="e14d18"; angle_reg_hex[2]="f2001c"; angle_reg_hex[3]="f2001c";}
        else if (degrees == "DEG_168_7"){angle_reg_hex[0]="f1ae9c"; angle_reg_hex[1]="f1ae9c"; angle_reg_hex[2]="fa001e"; angle_reg_hex[3]="fa001e";}
        else if (degrees == "DEG_180"){angle_reg_hex[0]="020020"; angle_reg_hex[1]="020020"; angle_reg_hex[2]="020020"; angle_reg_hex[3]="020020";}
    }
    else if (direction == "RIGHT"){
	if (degrees == "DEG_0"){angle_reg_hex[0]="820000"; angle_reg_hex[1]="820000"; angle_reg_hex[2]="000820"; angle_reg_hex[3]="000820";}
        else if (degrees == "DEG_11_25"){angle_reg_hex[0]="8a2000"; angle_reg_hex[1]="9a6104"; angle_reg_hex[2]="1049a6"; angle_reg_hex[3]="0008a2";}
        else if (degrees == "DEG_22_25"){angle_reg_hex[0]="924000"; angle_reg_hex[1]="b2c208"; angle_reg_hex[2]="208b2c"; angle_reg_hex[3]="000924";}
        else if (degrees == "DEG_33_75"){angle_reg_hex[0]="9a6000"; angle_reg_hex[1]="cb230c"; angle_reg_hex[2]="30ccb2"; angle_reg_hex[3]="0009a6";}
        else if (degrees == "DEG_45"){angle_reg_hex[0]="a28000"; angle_reg_hex[1]="e38410"; angle_reg_hex[2]="410e38"; angle_reg_hex[3]="000a28";}
        else if (degrees == "DEG_56_25"){angle_reg_hex[0]="aaa000"; angle_reg_hex[1]="fbe514"; angle_reg_hex[2]="514fbe"; angle_reg_hex[3]="000aaa";}
        else if (degrees == "DEG_67_5"){angle_reg_hex[0]="b2c000"; angle_reg_hex[1]="104618"; angle_reg_hex[2]="618104"; angle_reg_hex[3]="000b2c";}
        else if (degrees == "DEG_78_75"){angle_reg_hex[0]="bae000"; angle_reg_hex[1]="28a71c"; angle_reg_hex[2]="71c28a"; angle_reg_hex[3]="000bae";}
        else if (degrees == "DEG_90"){angle_reg_hex[0]="c30000"; angle_reg_hex[1]="410820"; angle_reg_hex[2]="820410"; angle_reg_hex[3]="000c30";}
        else if (degrees == "DEG_101_2"){angle_reg_hex[0]="cb2000"; angle_reg_hex[1]="596924"; angle_reg_hex[2]="924596"; angle_reg_hex[3]="000cb2";}
        else if (degrees == "DEG_112_5"){angle_reg_hex[0]="d34000"; angle_reg_hex[1]="71ca28"; angle_reg_hex[2]="a2871c"; angle_reg_hex[3]="000d34";}
        else if (degrees == "DEG_123_7"){angle_reg_hex[0]="db6000"; angle_reg_hex[1]="8a2b2c"; angle_reg_hex[2]="b2c8a2"; angle_reg_hex[3]="000db6";}
        else if (degrees == "DEG_135"){angle_reg_hex[0]="e38000"; angle_reg_hex[1]="a28c30"; angle_reg_hex[2]="c30a28"; angle_reg_hex[3]="000e38";}
        else if (degrees == "DEG_146_2"){angle_reg_hex[0]="eba000"; angle_reg_hex[1]="baed34"; angle_reg_hex[2]="d34bae"; angle_reg_hex[3]="000eba";}
        else if (degrees == "DEG_157_5"){angle_reg_hex[0]="f3c000"; angle_reg_hex[1]="d34e38"; angle_reg_hex[2]="e38d34"; angle_reg_hex[3]="000f3c";}
        else if (degrees == "DEG_168_7"){angle_reg_hex[0]="fbe000"; angle_reg_hex[1]="ebaf3c"; angle_reg_hex[2]="f3ceba"; angle_reg_hex[3]="000fbe";}
        else if (degrees == "DEG_180"){angle_reg_hex[0]="000000"; angle_reg_hex[1]="000000"; angle_reg_hex[2]="000000"; angle_reg_hex[3]="000000";}
    }
    else if (direction == "LEFT"){
	if (degrees == "DEG_0"){angle_reg_hex[0]="000820"; angle_reg_hex[1]="000820"; angle_reg_hex[2]="820000"; angle_reg_hex[3]="820000";}
        else if (degrees == "DEG_11_25"){angle_reg_hex[0]="1049a6"; angle_reg_hex[1]="0008a2"; angle_reg_hex[2]="8a2000"; angle_reg_hex[3]="9a6104";}
        else if (degrees == "DEG_22_25"){angle_reg_hex[0]="208b2c"; angle_reg_hex[1]="000924"; angle_reg_hex[2]="924000"; angle_reg_hex[3]="b2c208";}
        else if (degrees == "DEG_33_75"){angle_reg_hex[0]="30ccb2"; angle_reg_hex[1]="0009a6"; angle_reg_hex[2]="9a6000"; angle_reg_hex[3]="cb230c";}
        else if (degrees == "DEG_45"){angle_reg_hex[0]="410e38"; angle_reg_hex[1]="000a28"; angle_reg_hex[2]="a28000"; angle_reg_hex[3]="e38410";}
        else if (degrees == "DEG_56_25"){angle_reg_hex[0]="514fbe"; angle_reg_hex[1]="000aaa"; angle_reg_hex[2]="aaa000"; angle_reg_hex[3]="fbe514";}
        else if (degrees == "DEG_67_5"){angle_reg_hex[0]="618104"; angle_reg_hex[1]="000b2c"; angle_reg_hex[2]="b2c000"; angle_reg_hex[3]="104618";}
        else if (degrees == "DEG_78_75"){angle_reg_hex[0]="71c28a"; angle_reg_hex[1]="000bae"; angle_reg_hex[2]="bae000"; angle_reg_hex[3]="28a71c";}
        else if (degrees == "DEG_90"){angle_reg_hex[0]="820410"; angle_reg_hex[1]="000c30"; angle_reg_hex[2]="c30000"; angle_reg_hex[3]="410820";}
        else if (degrees == "DEG_101_2"){angle_reg_hex[0]="924596"; angle_reg_hex[1]="000cb2"; angle_reg_hex[2]="cb2000"; angle_reg_hex[3]="596924";}
        else if (degrees == "DEG_112_5"){angle_reg_hex[0]="a2871c"; angle_reg_hex[1]="000d34"; angle_reg_hex[2]="d34000"; angle_reg_hex[3]="71ca28";}
        else if (degrees == "DEG_123_7"){angle_reg_hex[0]="b2c8a2"; angle_reg_hex[1]="000db6"; angle_reg_hex[2]="db6000"; angle_reg_hex[3]="8a2b2c";}
        else if (degrees == "DEG_135"){angle_reg_hex[0]="c30a28"; angle_reg_hex[1]="000e38"; angle_reg_hex[2]="e38000"; angle_reg_hex[3]="a28c30";}
        else if (degrees == "DEG_146_2"){angle_reg_hex[0]="d34bae"; angle_reg_hex[1]="000eba"; angle_reg_hex[2]="eba000"; angle_reg_hex[3]="baed34";}
        else if (degrees == "DEG_157_5"){angle_reg_hex[0]="e38d34"; angle_reg_hex[1]="000f3c"; angle_reg_hex[2]="f3c000"; angle_reg_hex[3]="d34e38";}
        else if (degrees == "DEG_168_7"){angle_reg_hex[0]="f3ceba"; angle_reg_hex[1]="000fbe"; angle_reg_hex[2]="fbe000"; angle_reg_hex[3]="ebaf3c";}
        else if (degrees == "DEG_180"){angle_reg_hex[0]="000000"; angle_reg_hex[1]="000000"; angle_reg_hex[2]="000000"; angle_reg_hex[3]="000000";}
    }

    return angle_reg_hex; 
}


// DEGREES = [DEG_0,DEG_11_25,DEG_22_25,DEG_33_75,DEG_45,DEG_56_25,DEG_67_5,DEG_78_75,DEG_90,DEG_101_2,DEG_112_5,DEG_123_7,DEG_135,DEG_146_2,DEG_157_5,DEG_168_7,DEG_180];
