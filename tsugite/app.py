import sys
import time
import os

import PyQt5.QtWidgets as qtw
import PyQt5.QtGui as qtg
import PyQt5.QtCore as qtc

from main_window import MainWindow

class MovieSplashScreen(qtw.QSplashScreen):

    def __init__(self, movie):
        movie.jumpToFrame(0)
        pixmap = qtg.QPixmap(movie.frameRect().size())

        qtw.QSplashScreen.__init__(self, pixmap)
        self.movie = movie
        self.movie.frameChanged.connect(self.repaint)

    def showEvent(self, event):
        self.movie.start()

    def hideEvent(self, event):
        self.movie.stop()

    def paintEvent(self, event):
        painter = qtg.QPainter(self)
        pixmap = self.movie.currentPixmap()
        self.setMask(pixmap.mask())
        painter.drawPixmap(0, 0, pixmap)

    def sizeHint(self):
        return self.movie.scaledSize()


# deal with dpi
qtw.QApplication.setAttribute(qtc.Qt.AA_EnableHighDpiScaling, True)     # enable high dpi scaling
qtw.QApplication.setAttribute(qtc.Qt.AA_UseHighDpiPixmaps, True)        # use high dpi icons

app = qtw.QApplication(sys.argv)
movie = qtg.QMovie("img/tsugite_loading_3d.gif")

splash = MovieSplashScreen(movie)

splash.show()

start = time.time()

while movie.state() == qtg.QMovie.Running and time.time() < start + 1:
    app.processEvents()
# screen = app.screens()[0]
# dpi = screen.physicalDotsPerInch()

window = MainWindow()
window.show()
splash.finish(window)
sys.exit(app.exec_())
