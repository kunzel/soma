import pcl
from soma_io.soma_io import FileIO

pcd_folder = '/media/psf/strands_data_backup/20150505/patrol_run_10/room_6'

pcd_list = FileIO.scan_file(pcd_folder, '.pcd')
pcl.load(pcd_list[0])

