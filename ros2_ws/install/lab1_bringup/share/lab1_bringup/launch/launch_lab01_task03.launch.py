from launch import LaunchDescription 
from launch_ros.actions import Node 
# exactly this name! 
def generate_launch_description(): 
    #instantiate a LaunchDescription object 
    ld = LaunchDescription() 
    topic_remap = ['', '']      
    service_remap = ['', ''] 
      
    first_node = Node(package = 'lab01_task03',     
        executable = 'server',   
        name = 'DistanceServer',            
        output = 'screen',
    )
                                        
    second_node = Node(package = 'lab01_task03',
        executable = 'client',  
        name = 'DistanceClient',   
        output = 'screen', 
    ) 
    
    # add as much nodes needed for your application 
    # launch description 
    ld.add_action(first_node) 
    ld.add_action(second_node) 
    return ld 