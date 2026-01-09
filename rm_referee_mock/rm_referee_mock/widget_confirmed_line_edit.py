#!/usr/bin/env python3

from PyQt5.QtWidgets import QLineEdit


class ConfirmedLineEdit(QLineEdit):
    """一个自定义的 LineEdit，只在失去焦点或按回车时确认新值

    这样可以避免用户在输入过程中，中间状态的值就被读取。
    使用 get_confirmed_text() 获取已确认的值，而不是 text()
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._confirmed_text = ""
        # 当编辑完成时（失去焦点或按回车），确认新值
        self.editingFinished.connect(self._on_editing_finished)

    @classmethod
    def from_lineedit(cls, line_edit, default_text=""):
        """从现有的 QLineEdit 创建 ConfirmedLineEdit

        Args:
            line_edit: 要替换的 QLineEdit 实例
            default_text: 默认文本值

        Returns:
            ConfirmedLineEdit: 新创建的实例
        """
        # 创建新实例
        new_widget = cls(line_edit.parent())

        # 复制基本属性
        new_widget.setObjectName(line_edit.objectName())
        new_widget.setGeometry(line_edit.geometry())
        new_widget.setText(default_text)
        new_widget.set_confirmed_text(default_text)

        # 复制其他可能的属性
        new_widget.setEnabled(line_edit.isEnabled())
        new_widget.setVisible(line_edit.isVisible())
        new_widget.setReadOnly(line_edit.isReadOnly())
        new_widget.setPlaceholderText(line_edit.placeholderText())

        return new_widget

    def _on_editing_finished(self):
        """编辑完成时更新确认值"""
        self._confirmed_text = self.text().strip()

    def get_confirmed_text(self):
        """获取已确认的文本值"""
        return self._confirmed_text

    def set_confirmed_text(self, text):
        """设置文本并立即确认（用于程序更新，如计时器）"""
        self.setText(text)
        self._confirmed_text = text
