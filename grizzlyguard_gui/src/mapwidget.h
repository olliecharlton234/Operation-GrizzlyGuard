#ifndef MAPWIDGET_H
#define MAPWIDGET_H


#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QMap>

// A lightweight, dependency-free "map": it plots drone / bear positions as
// points on a local flat-earth projection centered on a home GPS coordinate,
// with a perimeter ring. No internet or tile server required.
//
// If you'd rather show real satellite/street map tiles, swap this widget
// for either:
//  - QWebEngineView loading a local HTML page with Leaflet.js + OpenStreetMap
//    tiles, and use QWebChannel to push marker updates from C++ to JS.
//  - Qt Location (QML: import QtLocation) embedded via QQuickWidget, using
//    MapQuickItem for markers. More "native" but heavier to set up.
class MapWidget : public QGraphicsView
{
    Q_OBJECT

public:
    explicit MapWidget(QWidget *parent = nullptr);

    // Sets the reference point (e.g. launch site / geofence center) that
    // all positions are plotted relative to.
    void setHomePosition(double latitude, double longitude);

    // Perimeter radius in meters, drawn as a ring around home.
    void setPerimeterRadius(double meters);

    void setDronePosition(double latitude, double longitude, double headingDegrees = 0.0);

    // id lets you add/move/remove individual bears by identity.
    void upsertBear(const QString &bearId, double latitude, double longitude);
    void removeBear(const QString &bearId);

    // Returns distance in meters from a given lat/lon to the perimeter edge
    // (negative = already inside... use closestBearDistanceToPerimeter()
    // for the value you'll feed into the traffic light).
    double closestBearDistanceToPerimeter() const;

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    QPointF projectToScene(double latitude, double longitude) const;

    QGraphicsScene m_scene;
    double m_homeLat = 0.0;
    double m_homeLon = 0.0;
    double m_perimeterMeters = 300.0;
    double m_metersPerSceneUnit = 1.0; // scene units == meters, keep simple

    QGraphicsEllipseItem *m_perimeterItem = nullptr;
    QGraphicsEllipseItem *m_droneItem = nullptr;
    QMap<QString, QGraphicsEllipseItem *> m_bearItems;
    QMap<QString, QPointF> m_bearLatLon;
};



#endif // MAPWIDGET_H
