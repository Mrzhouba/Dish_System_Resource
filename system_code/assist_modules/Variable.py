base_path = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/pointnet2_pytorch/'
weight_path = base_path + 'dgcnn_log/dgcnn_all_dish_300/models/epoch_280_acc_0.872781_iou_0.753227.pth'
model_name = 'DGCNN'
channels = 3
nclasses = 3

HOST = "192.168.20.176"
PORT = 1883

ur10_pull_dish_tra_path = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/pull_dish_trajectory' \
                          '/xinchang_ur10_tilt_fts_trajectory.json'

push_file_path = ['/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/ur10_push_trajectory/left_up_fts_push.json',
                  '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/ur10_push_trajectory/right_up_fts_push1.json',
                  '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/ur10_push_trajectory/right_up_fts_push2.json',
                  '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/ur10_push_trajectory/right_bottom_fts_push.json',
                  '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/ur10_push_trajectory/left_bottom_fts_push.json', ]

ur3_get_bowl_plan = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/get_bowl_trajectory/ur3_to_bowl_trajectory.json'

ur3_pull_dish_plan = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/pull_dish_trajectory/xinchang_ur3_tilt_trajectory.json'
ur3_get_start_plan = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/get_bowl_trajectory/ur3_to_start_trajectory.json'
ur3_to_start_plan = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/get_bowl_trajectory/ur3_to_start_trajectory.json'
ur3_add_bowl_plan = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/add_bowl_trajectory/xinchang_ur3_to_get_bowl_plan.json'
ur3_to_device_up_plan_3 = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/add_bowl_trajectory/ur3_to_device_up_plan_3.json'
ur3_to_device_up_plan_4 = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/add_bowl_trajectory/ur3_to_device_up_plan_4.json'

ur10_pull_to_initial_plan = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/pull_dish_trajectory/ur10_pull_to_initial.json'
ur10_shake_plan = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/pull_dish_trajectory/ur10_doudong_trajectory.json'

UR3_Q1 = [-1.294065300618307, -0.4414284865008753, -1.7840340773211878, -0.8931124846087855, 1.5558310747146606, 4.68846321105957]
ur3_lower_loc = [-1.2514274756060997, -1.585510555897848, -2.2166693846331995, 0.6478713750839233, 1.4034786224365232, 4.688450813293458]
# UR3_lay_location_old = [0.6819053888320923, -1.6980231443988245, -1.0546048323260706, -0.8341596762286585, 1.464781641960144, 4.944712162017822]
# UR3_lay_location = [0.6511808633804321, -1.6002844015704554, -1.2214010397540491, -0.7672761122332972, 1.496059775352478, 4.931004047393799]
UR3_lay_location = [0.6238856315612793, -1.6131184736834925, -1.209212605153219, -0.7664020697223108, 1.5233219861984253, 4.918135643005371]

ur3_multi1_catch_bowl_ready_location = [-2.721154753361837, -1.643240753804342, -1.726382080708639, 0.2404012680053711, 1.5811971426010132, 4.726001739501953]
ur3_multi1_catch_bowl_location = [-2.720675532017843, -1.8840978781329554, -2.0482099691974085, 0.8029826879501343, 1.5793399810791016, 4.72450065612793]
ur3_multi1_to_device_up_plan_2 = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/add_bowl_trajectory/xinchang_multi1_ur3_to_device_up_plan_2_with_loc.json'

ur3_multi2_catch_bowl_ready_location = [-3.0613835493670862, -1.5367935339557093, -1.7715333143817347, 0.180672287940979, 1.5983432531356812, 4.733086109161377]
ur3_multi2_catch_bowl_location = [-3.0607009569751185, -1.8407767454730433, -2.175638977681295, 0.8888124227523804, 1.5959826707839966, 4.731116771697998]
ur3_multi2_to_device_up_plan_2 = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/add_bowl_trajectory/xinchang_multi2_ur3_to_device_up_plan_2_with_loc.json'

ur3_multi_catch_bowl_ready_location = [ur3_multi1_catch_bowl_ready_location, ur3_multi2_catch_bowl_ready_location]
ur3_multi_catch_bowl_location = [ur3_multi1_catch_bowl_location, ur3_multi2_catch_bowl_location]
ur3_multi_to_device_up_plan_2 = [ur3_multi1_to_device_up_plan_2, ur3_multi2_to_device_up_plan_2]


one_bowl_loc = [[0.6785475611686707, -2.143420998250143, -1.6524904409991663, 0.632731556892395, 1.4522337913513184, 4.710690498352051]]
two_bowl_loc = [[0.5609722137451172, -2.3851164023028772, -1.1958144346820276, 0.4179569482803345, 1.5704615116119385, 4.708253383636475],
                [0.854677140712738, -1.9196131865130823, -2.0202696959124964, 0.7756901979446411, 1.2757338285446167, 4.7146172523498535]]


one_bowl_plate_loc = [[0.6788475513458252, -2.114838425313131, -1.6350634733783167, 0.5867165327072144, 1.4520896673202515, 4.710919380187988]]
two_bowl_plate_loc = [[0.5605400204658508, -2.370214287434713, -1.1712940374957483, 0.378198504447937, 1.5703656673431396, 4.70826530456543],
                      [0.854988694190979, -1.8816078344928187, -2.000563923512594, 0.7178903818130493, 1.275506615638733, 4.714893817901611]]

one_bowl_up_loc = [[0.6785595417022705, -2.0568464438067835, -1.5816801230060022, 0.4753528833389282, 1.452713131904602, 4.71119499206543]]
two_bowl_up_loc = [[0.5609482526779175, -2.3224809805499476, -1.1171134153949183, 0.27687156200408936, 1.5707608461380005, 4.708649635314941],
                   [0.854533314704895, -1.8066008726703089, -1.9412077108966272, 0.583523154258728, 1.2763694524765015, 4.715278148651123]]

#without_belt
one_bowl_loc_without_belt = [[-0.017460171376363576, -1.9405248800860804, -1.8334639708148401, 0.5242000818252563, 1.60004460811615, 4.708709716796875]]
two_bowl_loc_without_belt = [[-0.016932312642232716, -2.281534020100729, -1.2173855940448206, 0.21133208274841309, 1.5963661670684814, 4.70758056640625],
                             [-0.041060749684469044, -1.7843707243548792, -2.0543468634234827, 0.5656224489212036,1.599265694618225, 4.7073163986206055]]

one_bowl_up_loc_without_belt = [[-0.017364327107564748, -1.8596304098712366, -1.7459309736834925, 0.4372440576553345, 1.6005356311798096, 4.70899772644043]]
two_bowl_up_loc_without_belt = [[-0.017148319874898732, -2.234865967427389, -1.115591828023092, 0.22003746032714844, 1.5968692302703857, 4.707941055297852],
                                [-0.041384522114888966, -1.680676285420553, -1.9469612280475062, 0.48579180240631104, 1.5998408794403076, 4.707880973815918]]



UR10_pull = [-0.4024770895587366, -1.6448457876788538, 1.7061357498168945, -1.9389751593219202, -2.5280235449420374, -0.9280403296100062]
UR10_Q1_to_left = [-0.1647556463824671, -1.4304211775409144, 0.9728412628173828, -1.6566603819476526, -1.4693673292743128, -2.499138418828146]
UR10_Q1 = [-0.16467219987978154, -1.430373493825094, 0.9728174209594727, -1.656503979359762, -1.4693554083453577, -0.9290836493121546]

# fts
# ur10_pull_ready_old = [-0.35688478151430303, -1.716071907673971, 1.7269563674926758, -2.0164244810687464,
#                    -2.4187827746020716, -2.913269583378927]
ur10_pull_ready = [-0.32805139223207647, -1.6344783941852015, 1.6260218620300293, -1.8218877951251429, -2.481445614491598, -2.900316301976339]
ur10_pull_ready_loc = [-0.147, -0.606, 0.860]
ur10_pull_ready_rpy = [-0.004, 0.181, -1.572]
ur10_pull_ready_angle_y = -90.083

ur10_pull_finish = [-0.4704821745501917, -1.8910229841815394, 1.959409236907959, -2.5220163504229944, -2.234630886708395, -0.8257945219623011]
ur10_doudong1 = [-0.47105724016298467, -1.8920171896563929, 1.8372983932495117, -2.399296585713522, -2.2345708052264612, -0.8247750441180628]

ur10_max_acceleration = 0.5
ur10_max_velocity = 0.5

ur3_max_acceleration = 0.5
ur3_max_velocity = 0.5

left_up_push_start_ready_fts = [-0.04484874406923467, -1.185540024434225, 1.3968615531921387, -2.80781060854067,
                                -1.225170914326803, -1.4949610869037073]
right_up_push_start_ready_fts = [0.044309426099061966, -1.2078092733966272, 1.574920654296875, -3.0831645170794886,
                                 -1.8282392660724085, -0.31585961977113897]

camera_angele = 0.552
empty_depth = 0.5995836555099304

# require_volume = 3000
require_volume = {'RouPiHuangDou': 2000, 'JiMiHua': 2500, 'ChaoPuGua': 2300, 'ZhaTuDou': 2500, 'LaZiJi': 2500}
real_max_depth = 0.055

cut_safe_area_xbound_min = 0.082
cut_safe_area_xbound_max = 0.52
cut_safe_area_ybound_min = -0.18
# cut_safe_area_ybound_max = 0.11
cut_safe_area_ybound_max = 0.1
cut_safe_area_z = 0.5

pause_usb_name = "/dev/pause_usb"
order_usb_name = "/dev/order_usb"


ur3_middle_to_get_bowl_set = [-2.0130727926837366, -1.8026264349566858, -1.9296529928790491, 0.5835472345352173, 1.5170305967330933, 4.707821369171143]
ur3_end_to_get_bowl_set = [-1.2506488005267544, -1.518882099782125, -2.136658970509664, 0.5156739950180054, 1.401093363761902, 4.6888952255249015]




