#ifndef QHEARTBEAT_BLINKER
#define QHEARTBEAT_BLINKER

#include <QWidget>
#include <QTimer>

class HeartbeatBlinker : public QWidget
{
    Q_OBJECT

  public:
    HeartbeatBlinker(QWidget* parent_ = nullptr);
    void setPeriod(int period_);

    enum class eState
    {
        ALIVE,
        DEAD,
        DISCONNECTED
    };

  protected:
    void paintEvent(QPaintEvent* /*event*/) override;
    QSize sizeHint() const override
    {
        return {48, 48};
    }
    QSize minimumSizeHint() const override
    {
        return {24, 24};
    }

  public slots:
    void toggleBlinkImplementation(eState state_);

  private slots:
    void toggleBlinkStep();

  private:
    QTimer _timer;
    int _blinkPeriod = 500;
    eState _blinkState = eState::DISCONNECTED;
    bool _blinkStep = false;
};

#endif  // HEARTBEAT_BLINKER