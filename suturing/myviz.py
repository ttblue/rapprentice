#!/usr/bin/env python
import roslib; roslib.load_manifest('rviz')
import sys

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

import rviz

class MyViz( QWidget ):

    def __init__(self):
        QWidget.__init__(self)

        self.frame = rviz.VisualizationFrame()
        self.frame.initialize()
        self.setWindowTitle( "Rviz" )

        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )

        layout = QVBoxLayout()
        layout.addWidget( self.frame )
                
        h_layout = QHBoxLayout()
        
        top_button = QPushButton( "Top View" )
        top_button.clicked.connect( self.onTopButtonClick )
        h_layout.addWidget( top_button )
        
        side_button = QPushButton( "Side View" )
        side_button.clicked.connect( self.onSideButtonClick )
        h_layout.addWidget( side_button )
        
        layout.addLayout( h_layout )
        
        self.setLayout( layout )


    def onTopButtonClick( self ):
        self.switchToView( "Top View" );
        
    def onSideButtonClick( self ):
        self.switchToView( "Side View" );

    def switchToView( self, view_name ):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )

