Search.setIndex({envversion:46,filenames:["index","stubs/mobrob_behcon.behaviours.beh_align.BehAlign","stubs/mobrob_behcon.behaviours.beh_limfor.BehLimFor","stubs/mobrob_behcon.behaviours.beh_stop.BehStop","stubs/mobrob_behcon.behaviours.beh_transvel.BehConstTransVel","stubs/mobrob_behcon.behaviours.beh_turn.BehTurn","stubs/mobrob_behcon.behaviours.behaviour.Behaviour","stubs/mobrob_behcon.behaviours.behaviourgroup.BehaviourGroup","stubs/mobrob_behcon.connectors.camera.CameraTCP","stubs/mobrob_behcon.connectors.egopose.EgoPose","stubs/mobrob_behcon.connectors.gripper.Gripper","stubs/mobrob_behcon.connectors.laserscanner.LaserScanner","stubs/mobrob_behcon.core.perceptual_space.PerceptualSpace","stubs/mobrob_behcon.core.resolver.Resolver","stubs/mobrob_behcon.desires.desire.Desire","stubs/mobrob_behcon.desires.desires.DesCmdVel","stubs/mobrob_behcon.strategies.rd_strategy.RDStrategy","stubs/mobrob_behcon.strategies.strategy.Strategy"],objects:{"mobrob_behcon.behaviours.beh_align":{BehAlign:[1,0,1,""]},"mobrob_behcon.behaviours.beh_align.BehAlign":{"__init__":[1,1,1,""],add_desire:[1,1,1,""],error:[1,1,1,""],fire:[1,1,1,""],set_current_beh_group:[1,1,1,""],set_percept_space:[1,1,1,""],set_priority:[1,1,1,""],set_resolver:[1,1,1,""],success:[1,1,1,""]},"mobrob_behcon.behaviours.beh_limfor":{BehLimFor:[2,0,1,""]},"mobrob_behcon.behaviours.beh_limfor.BehLimFor":{"__init__":[2,1,1,""],add_desire:[2,1,1,""],error:[2,1,1,""],fire:[2,1,1,""],set_current_beh_group:[2,1,1,""],set_percept_space:[2,1,1,""],set_priority:[2,1,1,""],set_resolver:[2,1,1,""],success:[2,1,1,""]},"mobrob_behcon.behaviours.beh_stop":{BehStop:[3,0,1,""]},"mobrob_behcon.behaviours.beh_stop.BehStop":{"__init__":[3,1,1,""],add_desire:[3,1,1,""],error:[3,1,1,""],fire:[3,1,1,""],set_current_beh_group:[3,1,1,""],set_percept_space:[3,1,1,""],set_priority:[3,1,1,""],set_resolver:[3,1,1,""],success:[3,1,1,""]},"mobrob_behcon.behaviours.beh_transvel":{BehConstTransVel:[4,0,1,""]},"mobrob_behcon.behaviours.beh_transvel.BehConstTransVel":{"__init__":[4,1,1,""],add_desire:[4,1,1,""],error:[4,1,1,""],fire:[4,1,1,""],set_current_beh_group:[4,1,1,""],set_percept_space:[4,1,1,""],set_priority:[4,1,1,""],set_resolver:[4,1,1,""],success:[4,1,1,""]},"mobrob_behcon.behaviours.beh_turn":{BehTurn:[5,0,1,""]},"mobrob_behcon.behaviours.beh_turn.BehTurn":{"__init__":[5,1,1,""],add_desire:[5,1,1,""],error:[5,1,1,""],fire:[5,1,1,""],normalize_angle:[5,2,1,""],set_current_beh_group:[5,1,1,""],set_percept_space:[5,1,1,""],set_priority:[5,1,1,""],set_resolver:[5,1,1,""],success:[5,1,1,""]},"mobrob_behcon.behaviours.behaviour":{Behaviour:[6,0,1,""]},"mobrob_behcon.behaviours.behaviour.Behaviour":{"__init__":[6,1,1,""],add_desire:[6,1,1,""],error:[6,1,1,""],fire:[6,1,1,""],set_current_beh_group:[6,1,1,""],set_percept_space:[6,1,1,""],set_priority:[6,1,1,""],set_resolver:[6,1,1,""],success:[6,1,1,""]},"mobrob_behcon.behaviours.behaviourgroup":{BehaviourGroup:[7,0,1,""]},"mobrob_behcon.behaviours.behaviourgroup.BehaviourGroup":{"__init__":[7,1,1,""],activate_exclusive:[7,1,1,""],add:[7,1,1,""],deactivate:[7,1,1,""],set_error:[7,1,1,""],set_percept_space:[7,1,1,""],set_resolver:[7,1,1,""],set_success:[7,1,1,""]},"mobrob_behcon.connectors.camera":{CameraTCP:[8,0,1,""]},"mobrob_behcon.connectors.camera.CameraTCP":{"__init__":[8,1,1,""],get_frame:[8,1,1,""],recvall:[8,2,1,""],stringToRGB:[8,2,1,""]},"mobrob_behcon.connectors.egopose":{EgoPose:[9,0,1,""]},"mobrob_behcon.connectors.egopose.EgoPose":{"__init__":[9,1,1,""],callbackOdometry:[9,1,1,""],callbackPose:[9,1,1,""],get_current_pose:[9,1,1,""],get_current_time:[9,2,1,""]},"mobrob_behcon.connectors.gripper":{Gripper:[10,0,1,""]},"mobrob_behcon.connectors.gripper.Gripper":{"__init__":[10,1,1,""],getPosGRIP:[10,1,1,""],getPosVERT:[10,1,1,""],initGRIP:[10,1,1,""],initVERT:[10,1,1,""],moveAbsGRIP:[10,1,1,""],moveAbsVERT:[10,1,1,""],moveRelGRIP:[10,1,1,""],moveRelVERT:[10,1,1,""],refreshServo:[10,1,1,""],send_command:[10,1,1,""],setActivServo:[10,1,1,""],setSpeedGRIP:[10,1,1,""],setSpeedVERT:[10,1,1,""],setStayActivStepper:[10,1,1,""],stopAll:[10,1,1,""],stopGRIP:[10,1,1,""],stopVERT:[10,1,1,""],writeServo:[10,1,1,""]},"mobrob_behcon.connectors.laserscanner":{LaserScanner:[11,0,1,""]},"mobrob_behcon.connectors.laserscanner.LaserScanner":{"__init__":[11,1,1,""],calc_dist:[11,2,1,""],callback:[11,1,1,""],check_box:[11,1,1,""],draw_laserpoints:[11,1,1,""],extract_points:[11,2,1,""],filter_points:[11,2,1,""],get_current_time:[11,2,1,""],get_lst_scan_points:[11,1,1,""],get_x:[11,2,1,""],get_y:[11,2,1,""]},"mobrob_behcon.core.perceptual_space":{PerceptualSpace:[12,0,1,""]},"mobrob_behcon.core.perceptual_space.PerceptualSpace":{"__init__":[12,1,1,""],add_camera:[12,1,1,""],add_egopose:[12,1,1,""],add_laserscanner:[12,1,1,""]},"mobrob_behcon.core.resolver":{Resolver:[13,0,1,""]},"mobrob_behcon.core.resolver.Resolver":{"__init__":[13,1,1,""],add_desire:[13,1,1,""],get_current_time:[13,2,1,""],reset:[13,1,1,""],resetDesires:[13,1,1,""],resolveDesire:[13,1,1,""],runOnce:[13,1,1,""],set_behaviour_lst:[13,1,1,""]},"mobrob_behcon.desires.desire":{Desire:[14,0,1,""]},"mobrob_behcon.desires.desire.Desire":{"__init__":[14,1,1,""],set_priority:[14,1,1,""]},"mobrob_behcon.desires.desires":{DesCmdVel:[15,0,1,""]},"mobrob_behcon.desires.desires.DesCmdVel":{"__init__":[15,1,1,""],set_priority:[15,1,1,""]},"mobrob_behcon.strategies.rd_strategy":{RDStrategy:[16,0,1,""]},"mobrob_behcon.strategies.rd_strategy.RDStrategy":{"__init__":[16,1,1,""],activate_exclusive:[16,1,1,""],add_behgrp:[16,1,1,""],is_finished:[16,1,1,""],plan:[16,1,1,""],set_node:[16,1,1,""]},"mobrob_behcon.strategies.strategy":{Strategy:[17,0,1,""]},"mobrob_behcon.strategies.strategy.Strategy":{"__init__":[17,1,1,""],activate_exclusive:[17,1,1,""],add_behgrp:[17,1,1,""],is_finished:[17,1,1,""],plan:[17,1,1,""],set_node:[17,1,1,""]}},objnames:{"0":["py","class","Python class"],"1":["py","method","Python method"],"2":["py","staticmethod","Python static method"]},objtypes:{"0":"py:class","1":"py:method","2":"py:staticmethod"},terms:{"__init__":[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17],"abstract":[6,17],"byte":8,"class":[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17],"default":[1,2,3,4,5,6,7],"float":[1,2,3,4,5,9,11,15],"function":[5,6,7,8,9,10,11,13,14],"int":[1,2,3,4,5,6,7,8,9,12],"new":[8,13,14],"return":[5,8,9,11,12,13,14,15,16,17],"static":[5,8,9,11,13,14],"true":[16,17],about:[1,2,3,4,5,6,7,12],accept:14,activ:[1,2,3,4,5,6,7,13,16,17],activate_exclus:[7,16,17],actuat:[6,13],add:[7,12,13,16,17],add_behgrp:[16,17],add_camera:12,add_desir:[1,2,3,4,5,6,13],add_egopos:12,add_laserscann:12,address:[8,12],align:1,all:[7,11,12,13],also:[13,14],angl:[1,5,10,11],angular:15,ani:[3,14],approach:[2,3],arrai:[13,14],attribut:[8,14,15,16,17],base64:8,base64_str:8,been:[16,17],befor:[16,17],beh_align:1,beh_group:[1,2,3,4,5,6],beh_limfor:2,beh_nod:[16,17],beh_stop:3,beh_transvel:4,beh_turn:5,behaviourgroup:[1,2,3,4,5,6],behconnod:[16,17],behgrp:[16,17],blob:12,bound:11,bounderi:11,box:11,built:17,bytearrai:8,calc_dist:11,calcul:[11,13],call:[1,2,3,4,5,6,17],callback:[9,11],callbackodometri:9,callbackpos:9,camera:[8,12],can:[1,2,3,4,5,6,7,12,13,14],chang:[1,2,3,4,5,6],check:11,check_box:11,child:[1,2,3,4,5,17],cmd_vel:15,code:[1,2,3,4,5,6,7],com:12,command:10,configur:12,consist:[14,17],constant:4,construct:[13,14],constructor:[1,2,3,4,5,6,7,8,9,10,11],contain:[0,12],control:[10,15],convert:8,coordin:11,corner:11,correspond:13,current:[1,2,3,4,5,6,8,9,11,12,13],cycl:[1,2,3,4,5,13,17],cyclic:6,data:[8,9,11,12],deactiv:7,defin:6,deg:[4,5,11],degre:5,depend:[6,13,17],descmdvel:13,descript:[1,2,3,4,5,6],detail:[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17],differ:17,direct:4,distanc:[2,3,11],dock:16,down:2,draw:11,draw_laserpoint:11,drive:4,dure:[1,2,3,4,5,6],dynam:14,els:[16,17],encod:8,end:[1,2,3,4,5,6],eof:8,error:[1,2,3,4,5,6,7],everi:[1,2,3,4,5,17],execut:14,extract:11,extract_point:11,fals:[16,17],filter:11,filter_point:11,finish:[16,17],fire:[1,2,3,4,5,6],first:11,follow:[12,13,15],forward:[6,14],frame:8,from:[3,6,8,11,13],front:2,get:[8,9,11,12,13],get_current_pos:9,get_current_tim:[9,11,13],get_fram:8,get_i:11,get_lst_scan_point:11,get_x:11,getposgrip:10,getposvert:10,github:12,given:[1,2,3,4,5,6,7,11,13,16,17],group:[1,2,3,4,5,6,7,16,17],hase:15,helper:8,hit:8,http:12,imag:8,implement:17,index:0,inform:[1,2,3,4,5,6,7,12],inherit:[6,7,17],initgrip:10,initvert:10,ip_address:[8,12],is_finish:[16,17],item:13,laser:11,laserscan:[11,12],latest:11,length:11,linear:15,list:[11,13,14],loop:6,lower:11,lst_behaviour:13,lst_desir:13,map:15,master:12,messag:[9,11,13,15],method:[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17],millisecond:[9,11,13],mobrob:12,mobrob_pi_script:12,more:[1,2,3,4,5,6,7],move:4,moveabsgrip:10,moveabsvert:10,movement:15,moverelgrip:10,moverelvert:10,msg:[9,11],multipl:10,myrobot_model:[9,10,12],name:[1,2,3,4,5,6,7],nav_msg:[9,10,12],nearest:11,necessari:[1,2,3,4,5],need:[6,12,14,16,17],network:12,node:[16,17],none:[8,14],normal:5,normalize_angl:5,noth:[12,13,14,15,16,17],number:[8,14],object:[1,2,3,4,5,6,7,13,14],obstacl:[2,3,11],odom:[9,10,12],odometri:[9,10,12],opencv:8,option:[1,2,3,4,5,6,7,14],origin:11,orthogon:1,other:12,output:13,overview:0,page:0,paramet:[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17],parent:15,part:12,percept_spac:[1,2,3,4,5,6,7],perceptual_spac:12,perceptualspac:[1,2,3,4,5,6,7],plan:[16,17],point:11,polar:11,poll:[1,2,3,4,5,6,17],port:[8,12],pose:[9,10,12],posit:9,possibl:[9,10,12],prioriti:[1,2,3,4,5,6,7,14,15],program:12,provid:[8,9,10,11,12],publish:13,rad:5,radiu:11,rais:[1,2,3,4,5,6],rang:[5,11],rd_strategi:16,receiv:[9,11],recv:8,recval:8,refer:[1,2,3,4,5,6,7,12],refreshservo:10,relev:[1,2,3,4,5,6,7],repres:[1,2,3,4,5,8,9,10,11,15],reset:13,resetdesir:13,resolv:[1,2,3,4,5,6,7],resolvedesir:13,robot:[1,2,3,4,5,8,9,10,11,12,15],ros_servic:10,ros_top:[9,11,12],rot_vel:[1,5],rotat:[1,5],run:[12,13],runonc:13,runtim:[1,2,3,4,5,6],same:13,scan:12,scanpoint:11,search:0,second:11,sector:2,send:[1,2,3,4,5,6],send_command:10,sensor:12,sensor_msg:11,server:[8,12],servic:10,set:[1,2,3,4,5,6,7,13,14,15,16,17],set_behaviour_lst:13,set_current_beh_group:[1,2,3,4,5,6],set_error:7,set_nod:[16,17],set_percept_spac:[1,2,3,4,5,6,7],set_prior:[1,2,3,4,5,6,14,15],set_resolv:[1,2,3,4,5,6,7],set_success:7,setactivservo:10,setspeedgrip:10,setspeedvert:10,setstayactivstepp:10,setter:[14,15],sever:0,should:[2,3,4,5],side:3,similar:[13,15],slow:2,slowdist:2,slowspe:2,sock:8,socket:8,some:[1,2,3,4,5,6,7,11],sourc:[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17],speed:[1,2,4,5],sroup:[16,17],state:[7,16,17],statemachin:[16,17],statu:17,stop:[2,3],stopal:10,stopdist:[2,3],stopgrip:10,stopvert:10,stratgi:[16,17],strength:[6,14,15],string:[1,2,3,4,5,6,7,8,9,10,11,12],stringtorgb:8,success:[1,2,3,4,5,6,7],support:12,tcp:12,tcpserver:8,techniccontrol:12,them:7,thi:[0,1,2,3,4,5,6,7,11,12,17],those:17,three:15,time:[9,11,13],toler:1,topic:[9,10,11,12],trans_dir:4,trans_vel:4,trigger:7,tupl:15,turn90:16,turn:5,type:[1,2,3,4,5,6,9,11,13,16,17],understand:13,updat:17,upper:11,valu:[10,13,14,15],videotcpserver8_first_captur:12,visu:[11,12],visualis:11,wall:1,when:[3,14],where:[16,17],which:[1,2,3,4,5,6,13,17],writeservo:10,x_max:11,x_min:11,y_max:11,y_min:11,yaw:9,you:6},titles:["Welcome to <strong>MobRob_ROS_behcon</strong>&#8216;s documentation!","BehAlign","BehLimFor","BehStop","BehConstTransVel","BehTurn","Behaviour","BehaviourGroup","CameraTCP","EgoPose","Gripper","LaserScanner","PerceptualSpace","Resolver","Desire","DesCmdVel","RDStrategy","Strategy"],titleterms:{behalign:1,behaviour:[0,6],behaviourgroup:7,behconsttransvel:4,behlimfor:2,behstop:3,behturn:5,cameratcp:8,connector:0,core:0,descmdvel:15,desir:[0,14],document:0,egopos:9,gripper:10,indic:0,laserscann:11,mobrob_behcon:0,mobrob_ros_behcon:0,modul:0,perceptualspac:12,rdstrategi:16,resolv:13,strategi:[0,17],tabl:0,welcom:0}})