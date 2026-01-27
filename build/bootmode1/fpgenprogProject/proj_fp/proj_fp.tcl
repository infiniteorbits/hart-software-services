open_project -project {/home/trajce-nikolov/Git/infiniteorbits/hart-software-services/build/bootmode1/fpgenprogProject/proj_fp/proj_fp.pro}
enable_device -name {target} -enable 1
set_programming_file -name {target} -file {/home/trajce-nikolov/Git/infiniteorbits/hart-software-services/build/bootmode1/fpgenprogProject/proj_fp/target.ppd}
set_programming_action -action {PROGRAM} -name {target}
run_selected_actions
save_project
close_project
