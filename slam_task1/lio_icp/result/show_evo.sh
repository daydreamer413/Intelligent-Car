evo_traj tum ndt.txt opt_icp.txt icp.txt gnss.txt --plot --plot_mode=xy
# evo_traj tum ndt.txt opt_icp.txt icp.txt gnss.txt --save_as_kitti
# echo "gnss真值和icp_opt里程计之间绝对差异"
# evo_ape kitti gnss.kitti opt_icp.kitti --plot --plot_mode=xy --save_results gnss_opt_icp.zip
# clear
# echo "gnss真值和icp里程计之间绝对差异"
# evo_ape kitti gnss.kitti icp.kitti --plot --plot_mode=xy --save_results gnss_icp.zip
# clear
# echo "gnss真值和ndt里程计之间绝对差异"
# evo_ape kitti gnss.kitti ndt.kitti --plot --plot_mode=xy --save_results gnss_ndt.zip
# clear
# echo "几种方法之间绝对差异"
# evo_res gnss_opt_icp.zip gnss_icp.zip  gnss_ndt.zip