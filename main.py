<<<<<<< HEAD
import sys
from PyQt6.QtWidgets import QApplication

from gui import MainWindow


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
=======
import sys
from PyQt6.QtWidgets import QApplication

from gui import MainWindow


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
>>>>>>> 1f5935a291bc6e80918cf3a2c91adbc8e9a8f452
    sys.exit(app.exec())