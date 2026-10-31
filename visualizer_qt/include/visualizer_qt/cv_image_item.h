#pragma once

#include <QImage>
#include <QQuickPaintedItem>

namespace visualizer_qt
{

// Displays a QImage scaled-to-fit (aspect preserved, letterboxed, never
// cropped) inside the item's bounds — same fix the ImGui app needed when
// drawing at native texture size overflowed the panel box.
class CvImageItem : public QQuickPaintedItem
{
  Q_OBJECT
  Q_PROPERTY(QImage image READ image WRITE setImage NOTIFY imageChanged)

public:
  explicit CvImageItem(QQuickItem * parent = nullptr);

  QImage image() const {return image_;}
  void setImage(const QImage & img);

  void paint(QPainter * painter) override;

signals:
  void imageChanged();

private:
  QImage image_;
};

}  // namespace visualizer_qt
