from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, QGridLayout,
                             QLabel, QMessageBox, QRadioButton, QGroupBox, QLineEdit, QSlider)
from PyQt5.QtCore import Qt
from dynamixel import dynamixel


class DX_control_GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.dx = dynamixel(device_name='/dev/ttyUSB0', protocol_version=2.0)
        for i in range(4):
            self.dx.enable_torque(i, True)
            self.dx.enable_led(i, True)

        self.slider = []
        self.label = []
        self.runUI()

    def runUI(self):
        grid = QGridLayout()
        # index: number of row
        for i in range(4):
            label = QLabel('DX ' + str(i))
            label.setStyleSheet("color: black;" "font-size: 15px;")
            grid.addWidget(label, i, 0, 1, 1)   # arg=(widget, row, col, rowspan, colspan)

            self.slider.append(QSlider(Qt.Horizontal, self))
            self.slider[i].setRange(0, 4095)
            curr_pos = self.dx.get_pos(i)
            self.slider[i].setSliderPosition(curr_pos)
            self.slider[i].setSingleStep(1)
            self.slider[i].setTickPosition(QSlider.TicksBelow)
            self.slider[i].setTickInterval(1000)
            self.slider[i].valueChanged.connect(getattr(self, 'sliderValueChanged' + str(i)))
            grid.addWidget(self.slider[i], i, 1, 1, 1)

            self.label.append(QLabel(str(curr_pos)))
            self.label[i].setStyleSheet("color: black;" "font-size: 15px;")
            grid.addWidget(self.label[i], i, 2, 1, 1)

        # Sync motion
        row = 4
        label = QLabel('DX Sync')
        label.setStyleSheet("color: black;" "font-size: 15px;")
        grid.addWidget(label, row, 0, 1, 1)  # arg=(widget, row, col, rowspan, colspan)
        self.sliderSync = QSlider(Qt.Horizontal, self)
        self.sliderSync.setRange(0, 4095)

        init_pos_sync = int(4096/2)
        self.sliderSync.setSliderPosition(init_pos_sync)
        self.sliderSync.setSingleStep(1)
        self.sliderSync.setTickPosition(QSlider.TicksBelow)
        self.sliderSync.setTickInterval(1000)
        self.sliderSync.valueChanged.connect(self.sliderValueChangedSync)
        grid.addWidget(self.sliderSync, row, 1, 1, 1)
        self.labelSync = QLabel(str(init_pos_sync))
        self.labelSync.setStyleSheet("color: black;" "font-size: 15px;")
        grid.addWidget(self.labelSync, row, 2, 1, 1)

        # Center
        row = 5
        self.btnMoveCenter = QPushButton('Move to Center', self)
        self.btnMoveCenter.clicked.connect(self.btnMoveCenterClicked)
        grid.addWidget(self.btnMoveCenter, row, 2, 1, 1)

        self.setLayout(grid)
        self.setWindowTitle('DX_test_GUI')
        self.setGeometry(500, 150, 600, 400)
        self.show()

    def sliderValueChanged0(self, value):
        self.dx.set_pos(0, value)
        self.label[0].setText(str(value))

    def sliderValueChanged1(self, value):
        self.dx.set_pos(1, value)
        self.label[1].setText(str(value))

    def sliderValueChanged2(self, value):
        self.dx.set_pos(2, value)
        self.label[2].setText(str(value))

    def sliderValueChanged3(self, value):
        self.dx.set_pos(3, value)
        self.label[3].setText(str(value))

    def sliderValueChangedSync(self, value):
        self.dx.set_pos_sync([0, 1, 2, 3], [value, value, value, value])
        for i in range(4):
            self.slider[i].setSliderPosition(value)
            self.label[i].setText(str(value))
        self.labelSync.setText(str(value))
        self.sliderSync.setSliderPosition(value)

    def btnMoveCenterClicked(self):
        self.sliderValueChangedSync(value=int(4096/2))



if __name__ == '__main__':
    import sys
    app = QApplication(sys.argv)
    gui = DX_control_GUI()
    sys.exit(app.exec_())