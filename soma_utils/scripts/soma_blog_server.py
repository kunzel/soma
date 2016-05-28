#!/usr/bin/env python
import roslib; roslib.load_manifest("soma_utils")
import rospy
from rospkg import RosPack

import robblog.utils

if __name__ == '__main__':
    rospy.init_node("soma_blog_server")

    robblog_path =  roslib.packages.get_pkg_dir('soma_utils') 
    blog_collection = 'soma_blog'
    
    # where are the blog files going to be put
    blog_path = robblog_path + '/content'

    # initialise blog
    robblog.utils.init_blog(blog_path)
    proc = robblog.utils.serve(blog_path, 'localhost', '4040')
    
    try: 
        converter = robblog.utils.EntryConverter(blog_path=blog_path, collection=blog_collection)
        while not rospy.is_shutdown():
            # supply True convert to force all pages to be regenerated
            converter.convert()
            rospy.sleep(1)

    except Exception, e:
        rospy.logfatal(e)
    finally:
        proc.terminate()
