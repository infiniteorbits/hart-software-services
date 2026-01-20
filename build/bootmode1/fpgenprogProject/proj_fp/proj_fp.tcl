open_project -project {/home/trajce-nikolov/Git/hart-software-services/build/bootmode1/fpgenprogProject/proj_fp/proj_fp.pro} -connect_programmers {FALSE}
load_programming_data -name {target} -header {/home/trajce-nikolov/Git/hart-software-services/build/bootmode1/fpgenprogProject/target.hdr} -envm {/home/trajce-nikolov/Git/hart-software-services/build/bootmode1/fpgenprogProject/envm.efc} -spm {/home/trajce-nikolov/Git/hart-software-services/build/bootmode1/fpgenprogProject/target.spm} -dca {/home/trajce-nikolov/Git/hart-software-services/build/bootmode1/fpgenprogProject/target.dca}
export_single_ppd -name {target} -file {/home/trajce-nikolov/Git/hart-software-services/build/bootmode1/fpgenprogProject/proj_fp/target.ppd}
save_project
close_project
