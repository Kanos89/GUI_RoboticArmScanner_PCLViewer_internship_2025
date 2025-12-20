from PyQt6.QtWidgets import QDialog, QVBoxLayout, QCheckBox, QPushButton, QHBoxLayout, QLabel

class SettingsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Point Cloud Settings")
        self.setMinimumWidth(200)   
        self.setMinimumHeight(150)  

        layout = QVBoxLayout()

        # Instruction label
        layout.addWidget(QLabel("Select preprocessing options before visualization:"))

        # Checkboxes
        self.downsample_cb = QCheckBox("Downsample point cloud")
        self.normal_cb = QCheckBox("Estimate normals")      
        self.meshing_cb = QCheckBox("Apply Poisson meshing")

        layout.addWidget(self.downsample_cb)
        layout.addWidget(self.normal_cb)
        layout.addWidget(self.meshing_cb)

        # Buttons
        button_layout = QHBoxLayout()
        ok_btn = QPushButton("OK")
        cancel_btn = QPushButton("Cancel")
        button_layout.addWidget(ok_btn)
        button_layout.addWidget(cancel_btn)

        layout.addLayout(button_layout)
        self.setLayout(layout)

        # Connections
        ok_btn.clicked.connect(self.accept)
        cancel_btn.clicked.connect(self.reject)

    def get_settings(self):
        """Return a dict with user-selected settings."""
        return {
            "downsample": self.downsample_cb.isChecked(),
            "estimate_normals": self.normal_cb.isChecked(),
            "meshing": self.meshing_cb.isChecked(),
        }
