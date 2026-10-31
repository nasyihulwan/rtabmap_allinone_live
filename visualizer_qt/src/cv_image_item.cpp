#include "visualizer_qt/cv_image_item.h"

#include <algorithm>

#include <QPainter>

namespace visualizer_qt
{

CvImageItem::CvImageItem(QQuickItem * parent)
: QQuickPaintedItem(parent)
{
}

void CvImageItem::setImage(const QImage & img)
{
  image_ = img;
  emit imageChanged();
  update();
}

void CvImageItem::paint(QPainter * painter)
{
  if (image_.isNull() || width() <= 0 || height() <= 0) {
    return;
  }
  double scale = std::min(
    width() / static_cast<double>(image_.width()),
    height() / static_cast<double>(image_.height()));
  double draw_w = image_.width() * scale;
  double draw_h = image_.height() * scale;
  QPointF offset((width() - draw_w) / 2.0, (height() - draw_h) / 2.0);
  painter->drawImage(QRectF(offset.x(), offset.y(), draw_w, draw_h), image_);
}

}  // namespace visualizer_qt
