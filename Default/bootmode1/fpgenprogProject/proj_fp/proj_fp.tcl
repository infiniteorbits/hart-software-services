open_project -project {C:\Users\admin\abel\07-hart_services\hart-software-services\Default/bootmode1/fpgenprogProject\proj_fp\proj_fp.pro}
enable_device -name {target} -enable 1
set_programming_file -name {target} -file {C:\Users\admin\abel\07-hart_services\hart-software-services\Default/bootmode1/fpgenprogProject\proj_fp\target.ppd}
set_programming_action -action {PROGRAM} -name {target}
run_selected_actions
save_project
close_project
