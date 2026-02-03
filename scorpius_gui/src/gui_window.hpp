#ifndef GUI_WINDOW_HPP
#define GUI_WINDOW_HPP

#include <QMainWindow>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

class GuiWindow : public QMainWindow
{
    Q_OBJECT
  public:
    explicit GuiWindow(QWidget* parent = nullptr);

    int addTab(QWidget* page, const QString& label);

  private:
    QWidget* _central{nullptr};
    QVBoxLayout* _layout{nullptr};
    QTabWidget* _tabs{nullptr};
};

#endif  // define GUI_WINDOW_HPP