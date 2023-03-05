#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Marcos Maroto Gómez"
__copyright__ = "Embodied Emotion, Cognition and Inter Action Lab. University of Hertfordshire"
__credits__ = ["Marcos Maroto Gómez"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Marcos Maroto Gómez"
__email__ = "marmarot@ing.uc3m.es"
__status__ = "Development"


from datetime import datetime as dt

import os
import roslib
import rospkg
import rospy


pkg_name = "motivational_dms"
roslib.load_manifest(pkg_name)

# get the 'data' folder path
rospack = rospkg.RosPack()
PATH = rospack.get_path('motivational_dms') + "/data/log/"


class Logger():

    """
    Logger class.
    """

    def __init__(self, name, dirname, type=''):
        """
        Init method.
        """

        # set prefix
        prefix = ''
        if type != '':
            prefix = type + '-'

        # create filename
        dirpath = PATH + dirname + '/'
        filename = dirpath + prefix + name + '.txt'

        # create folder
        if not os.path.exists(os.path.dirname(filename)):
            try:
                os.makedirs(os.path.dirname(filename))
            except OSError:
                pass

        # open file
        self.__f = open(filename, 'a')

    def write_labels(self, data):
        """
        Write first line of the log file with labels.
        """

        try:
            self.__f.write(data + '\n')
        except ValueError:
            pass

    def write_data(self, data, gnuplot=False, timestamp=True):
        """
        Write data in the log file.
        """

        #if not gnuplot:
            #data = str(data).replace('.', ',')

        if timestamp:
            aux_str = '%s;%s\n' % (dt.now().strftime("%d/%m/%Y  %H:%M:%S.%f"), data)
        else:
            aux_str = '%s\n' %  data
        try:
            self.__f.write(aux_str)
        except ValueError:
            pass

    def close(self):
        """
        Close file.
        """

        self.__f.close()