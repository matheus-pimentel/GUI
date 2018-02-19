#include "scenemodifier.h"
#include "iostream"
#include <QtCore/QDebug>
#include <QMaterial>

using namespace std;

scenemodifier::scenemodifier(Qt3DCore::QEntity *rootEntity)
    : m_rootEntity(rootEntity)
{
    state = resize_matrix(4,3);
    old_state = resize_matrix(4,3);
    des_state = resize_matrix(5,3);
    old_des_state = resize_matrix(5,3);

    a1.resize(3,1);
    a2.resize(3,1);
    a3.resize(3,1);
    a1 << 1, 0, 0;
    a2 << 0, 1, 0;
    a3 << 0, 0, 1;
    b1.resize(1,3);
    b2.resize(1,3);
    b3.resize(1,3);

    this->createLines(QVector3D(0, 0, 0), QVector3D(0, 0, 1), 1, true, "");
    this->createLines(QVector3D(0, 0, 0), QVector3D(0, 1, 0), 1, true, "");
    this->createLines(QVector3D(0, 0, 0), QVector3D(1, 0, 0), 1, true, "");

}
scenemodifier::~scenemodifier()
{
}

void scenemodifier::createLines(const QVector3D &v0, const QVector3D &v1,
                                const unsigned int index, const bool axis, const QString &lod_param)
{
    Qt3DRender::QGeometryRenderer *line_mesh = new Qt3DRender::QGeometryRenderer();
    Qt3DRender::QGeometry *geometry = new Qt3DRender::QGeometry(line_mesh);

    Qt3DRender::QBuffer *vertexDataBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::VertexBuffer, geometry);
    Qt3DRender::QBuffer *indexDataBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::IndexBuffer, geometry);

    QByteArray vertexBufferData;
    vertexBufferData.resize(2 * (3 + 3) * sizeof(float));

    QVector<QVector3D> vertices = (!axis ? QVector<QVector3D>() << v0 << QVector3D (0.5f, 0.0f, 0.5f) << v1  << QVector3D (0.5f, 0.0f, 0.5f)
                                         : (index==0 ? QVector<QVector3D>() << v0 << QVector3D (1.0f, 0.0f, 0.0f) << v1  << QVector3D (1.0f, 0.0f, 0.0f)
                                                      : (index==1 ? QVector<QVector3D>() << v0 << QVector3D (0.0f, 0.0f, 0.0f) << v1  << QVector3D (0.0f, 0.0f, 0.0f)
                                                                  : (index==2 ? QVector<QVector3D>() << v0 << QVector3D (0.0f, 0.0f, 1.0f) << v1  << QVector3D (0.0f, 0.0f, 1.0f)
                                                                              : QVector<QVector3D>() << v0 << QVector3D (0.0f, 1.0f, 1.0f) << v1  << QVector3D (0.0f, 1.0f, 1.0f)))));

    float *rawVertexArray = reinterpret_cast<float *>(vertexBufferData.data());
    int idx = 0;

    Q_FOREACH (const QVector3D &v, vertices) {
        rawVertexArray[idx++] = v.x();
        rawVertexArray[idx++] = v.y();
        rawVertexArray[idx++] = v.z();
    }

    QByteArray indexBufferData;
    indexBufferData.resize(2 * 3 * sizeof(ushort));
    ushort *rawIndexArray = reinterpret_cast<ushort *>(indexBufferData.data());

    rawIndexArray[0] = 0;
    rawIndexArray[1] = 1;

    vertexDataBuffer->setData(vertexBufferData);
    indexDataBuffer->setData(indexBufferData);

    // Attributes
    Qt3DRender::QAttribute *positionAttribute = new Qt3DRender::QAttribute();
    positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    positionAttribute->setBuffer(vertexDataBuffer);
    positionAttribute->setDataType(Qt3DRender::QAttribute::Float);
    positionAttribute->setDataSize(3);
    positionAttribute->setByteOffset(0);
    positionAttribute->setByteStride(6 * sizeof(float));
    positionAttribute->setCount(2);
    positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());

    Qt3DRender::QAttribute *colorAttribute = new Qt3DRender::QAttribute();
    colorAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    colorAttribute->setBuffer(vertexDataBuffer);
    colorAttribute->setDataType(Qt3DRender::QAttribute::Float);
    colorAttribute->setDataSize(3);
    colorAttribute->setByteOffset(3 * sizeof(float));
    colorAttribute->setByteStride(6 * sizeof(float));
    colorAttribute->setCount(2);
    colorAttribute->setName(Qt3DRender::QAttribute::defaultColorAttributeName());

    Qt3DRender::QAttribute *indexAttribute = new Qt3DRender::QAttribute();
    indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
    indexAttribute->setBuffer(indexDataBuffer);
    indexAttribute->setDataType(Qt3DRender::QAttribute::UnsignedShort);
    indexAttribute->setDataSize(1);
    indexAttribute->setByteOffset(0);
    indexAttribute->setByteStride(0);
    indexAttribute->setCount(2);

    geometry->addAttribute(positionAttribute);
    geometry->addAttribute(colorAttribute);
    geometry->addAttribute(indexAttribute);

    line_mesh->setInstanceCount(1);
    line_mesh->setIndexOffset(0);
    line_mesh->setFirstInstance(0);
    line_mesh->setPrimitiveType(Qt3DRender::QGeometryRenderer::Lines);
    line_mesh->setGeometry(geometry);
    line_mesh->setVertexCount(2);

    // Material
    Qt3DRender::QMaterial *material = new Qt3DExtras::QPerVertexColorMaterial(m_rootEntity);

    // Line Entity
    m_lineEntity = new Qt3DCore::QEntity(m_rootEntity);
    m_lineEntity->addComponent(line_mesh);
    m_lineEntity->addComponent(material);

    if(!axis)
    {
        m_lineEntity->setObjectName(QString::number(index)+ ":" +lod_param);
        Qt3DRender::QObjectPicker createObjectPickerForEntity(m_lineEntity);
    }
}

void scenemodifier::create_trajectories()
{
    this->createLines(QVector3D(state.matrix[0][0], state.matrix[0][1], state.matrix[0][2]), QVector3D(old_state.matrix[0][0], old_state.matrix[0][1], old_state.matrix[0][2]), 2, true, "");
    this->createLines(QVector3D(des_state.matrix[0][0], des_state.matrix[0][1], des_state.matrix[0][2]), QVector3D(old_des_state.matrix[0][0], old_des_state.matrix[0][1], old_des_state.matrix[0][2]), 0, true, "");
}

void scenemodifier::set_params(params params_q)
{
    quad_params = params_q;
}

void scenemodifier::set_states(matrixds a, matrixds b, matrixds c, matrixds d)
{
    state = a;
    old_state = b;
    des_state = c;
    old_des_state = d;
}

void scenemodifier::init_plot()
{ 
    MatrixXd pos(1,3);
    MatrixXd motor1_pos(1,3), motor2_pos(1,3), motor3_pos(1,3), motor4_pos(1,3), up_pos(1,3);

    double roll = state.matrix[2][0];
    double pitch = state.matrix[2][1];
    double yaw = state.matrix[2][2];
    pos << state.matrix[0][0], state.matrix[0][1], state.matrix[0][2];

    b1 = (mds2mxd(rotation_matrix(roll,pitch,yaw))*a1).transpose();
    b2 = (mds2mxd(rotation_matrix(roll,pitch,yaw))*a2).transpose();
    b3 = (mds2mxd(rotation_matrix(roll,pitch,yaw))*a3).transpose();

    motor1_pos = pos+b1*quad_params.l;
    motor2_pos = pos+b2*quad_params.l;
    motor3_pos = pos-b1*quad_params.l;
    motor4_pos = pos-b2*quad_params.l;
    up_pos = pos + b3*quad_params.l/4;

    // motor1 shape data
    Qt3DExtras::QCylinderMesh *motor1 = new Qt3DExtras::QCylinderMesh();
    motor1->setRadius(quad_params.l/5);
    motor1->setLength(quad_params.l/20);
    motor1->setRings(100);
    motor1->setSlices(20);

    // motor1 Transform
    Qt3DCore::QTransform *motor1Transform = new Qt3DCore::QTransform();
    motor1Transform->setScale(1.0f);
    motor1Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor1Transform->setTranslation(QVector3D(motor1_pos(0,0),motor1_pos(0,1),motor1_pos(0,2)));

    Qt3DExtras::QPhongMaterial *motor1Material = new Qt3DExtras::QPhongMaterial();
    motor1Material->setDiffuse(QColor("Red"));

    // motor1
    quad_motor1 = new Qt3DCore::QEntity(m_rootEntity);
    quad_motor1->addComponent(motor1);
    quad_motor1->addComponent(motor1Material);
    quad_motor1->addComponent(motor1Transform);

    // motor2 shape data
    Qt3DExtras::QCylinderMesh *motor2 = new Qt3DExtras::QCylinderMesh();
    motor2->setRadius(quad_params.l/5);
    motor2->setLength(quad_params.l/20);
    motor2->setRings(100);
    motor2->setSlices(20);

    // motor2 Transform
    Qt3DCore::QTransform *motor2Transform = new Qt3DCore::QTransform();
    motor2Transform->setScale(1.0f);
    motor2Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor2Transform->setTranslation(QVector3D(motor2_pos(0,0),motor2_pos(0,1),motor2_pos(0,2)));

    Qt3DExtras::QPhongMaterial *motor2Material = new Qt3DExtras::QPhongMaterial();
    motor2Material->setDiffuse(QColor("Black"));

    // motor2
    quad_motor2 = new Qt3DCore::QEntity(m_rootEntity);
    quad_motor2->addComponent(motor2);
    quad_motor2->addComponent(motor2Material);
    quad_motor2->addComponent(motor2Transform);

    // motor3 shape data
    Qt3DExtras::QCylinderMesh *motor3 = new Qt3DExtras::QCylinderMesh();
    motor3->setRadius(quad_params.l/5);
    motor3->setLength(quad_params.l/20);
    motor3->setRings(100);
    motor3->setSlices(20);

    // motor3 Transform
    Qt3DCore::QTransform *motor3Transform = new Qt3DCore::QTransform();
    motor3Transform->setScale(1.0f);
    motor3Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor3Transform->setTranslation(QVector3D(motor3_pos(0,0),motor3_pos(0,1),motor3_pos(0,2)));

    Qt3DExtras::QPhongMaterial *motor3Material = new Qt3DExtras::QPhongMaterial();
    motor3Material->setDiffuse(QColor("Black"));

    // motor3
    quad_motor3 = new Qt3DCore::QEntity(m_rootEntity);
    quad_motor3->addComponent(motor3);
    quad_motor3->addComponent(motor3Material);
    quad_motor3->addComponent(motor3Transform);

    // motor4 shape data
    Qt3DExtras::QCylinderMesh *motor4 = new Qt3DExtras::QCylinderMesh();
    motor4->setRadius(quad_params.l/5);
    motor4->setLength(quad_params.l/20);
    motor4->setRings(100);
    motor4->setSlices(20);

    // motor4 Transform
    Qt3DCore::QTransform *motor4Transform = new Qt3DCore::QTransform();
    motor4Transform->setScale(1.0f);
    motor4Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor4Transform->setTranslation(QVector3D(motor4_pos(0,0),motor4_pos(0,1),motor4_pos(0,2)));

    Qt3DExtras::QPhongMaterial *motor4Material = new Qt3DExtras::QPhongMaterial();
    motor4Material->setDiffuse(QColor("Black"));

    // motor4
    quad_motor4 = new Qt3DCore::QEntity(m_rootEntity);
    quad_motor4->addComponent(motor4);
    quad_motor4->addComponent(motor4Material);
    quad_motor4->addComponent(motor4Transform);

    // arm1 shape data
    Qt3DExtras::QCylinderMesh *arm1 = new Qt3DExtras::QCylinderMesh();
    arm1->setRadius(quad_params.l/20);
    arm1->setLength(quad_params.l*2);
    arm1->setRings(100);
    arm1->setSlices(20);

    // arm1 Transform
    Qt3DCore::QTransform *arm1Transform = new Qt3DCore::QTransform();
    arm1Transform->setScale(1.0f);
    arm1Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2)),QVector3D(b3(0,0),b3(0,1),b3(0,2))));
    arm1Transform->setTranslation(QVector3D(pos(0,0),pos(0,1),pos(0,2)));

    Qt3DExtras::QPhongMaterial *arm1Material = new Qt3DExtras::QPhongMaterial();
    arm1Material->setDiffuse(QColor("Blue"));

    // arm1
    quad_arm1 = new Qt3DCore::QEntity(m_rootEntity);
    quad_arm1->addComponent(arm1);
    quad_arm1->addComponent(arm1Material);
    quad_arm1->addComponent(arm1Transform);

    // arm2 shape data
    Qt3DExtras::QCylinderMesh *arm2 = new Qt3DExtras::QCylinderMesh();
    arm2->setRadius(quad_params.l/20);
    arm2->setLength(quad_params.l*2);
    arm2->setRings(100);
    arm2->setSlices(20);

    // arm2 Transform
    Qt3DCore::QTransform *arm2Transform = new Qt3DCore::QTransform();
    arm2Transform->setScale(1.0f);
    arm2Transform->setRotation(QQuaternion::fromAxes(QVector3D(b2(0,0),b2(0,1),b2(0,2)),QVector3D(-b1(0,0),-b1(0,1),-b1(0,2)),QVector3D(b3(0,0),b3(0,1),b3(0,2))));
    arm2Transform->setTranslation(QVector3D(pos(0,0),pos(0,1),pos(0,2)));

    Qt3DExtras::QPhongMaterial *arm2Material = new Qt3DExtras::QPhongMaterial();
    arm2Material->setDiffuse(QColor("Blue"));

    // arm2
    quad_arm2 = new Qt3DCore::QEntity(m_rootEntity);
    quad_arm2->addComponent(arm2);
    quad_arm2->addComponent(arm2Material);
    quad_arm2->addComponent(arm2Transform);

    // up shape data
    Qt3DExtras::QCylinderMesh *up = new Qt3DExtras::QCylinderMesh();
    up->setRadius(quad_params.l/20);
    up->setLength(quad_params.l/2);
    up->setRings(100);
    up->setSlices(20);

    // up Transform
    Qt3DCore::QTransform *upTransform = new Qt3DCore::QTransform();
    upTransform->setScale(1.0f);
    upTransform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    upTransform->setTranslation(QVector3D(up_pos(0,0),up_pos(0,1),up_pos(0,2)));

    Qt3DExtras::QPhongMaterial *upMaterial = new Qt3DExtras::QPhongMaterial();
    upMaterial->setDiffuse(QColor("Blue"));

    // up
    quad_up = new Qt3DCore::QEntity(m_rootEntity);
    quad_up->addComponent(up);
    quad_up->addComponent(upMaterial);
    quad_up->addComponent(upTransform);
}

void scenemodifier::update_plot()
{
    create_trajectories();

    MatrixXd pos(1,3);
    MatrixXd motor1_pos(1,3), motor2_pos(1,3), motor3_pos(1,3), motor4_pos(1,3), up_pos(1,3);

    double roll = state.matrix[2][0];
    double pitch = state.matrix[2][1];
    double yaw = state.matrix[2][2];
    pos << state.matrix[0][0], state.matrix[0][1], state.matrix[0][2];

    b1 = (mds2mxd(rotation_matrix(roll,pitch,yaw))*a1).transpose();
    b2 = (mds2mxd(rotation_matrix(roll,pitch,yaw))*a2).transpose();
    b3 = (mds2mxd(rotation_matrix(roll,pitch,yaw))*a3).transpose();

    motor1_pos = pos+b1*quad_params.l;
    motor2_pos = pos+b2*quad_params.l;
    motor3_pos = pos-b1*quad_params.l;
    motor4_pos = pos-b2*quad_params.l;
    up_pos = pos + b3*quad_params.l/4;

    // motor1 Transform
    Qt3DCore::QTransform *motor1Transform = new Qt3DCore::QTransform();
    motor1Transform->setScale(1.0f);
    motor1Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor1Transform->setTranslation(QVector3D(motor1_pos(0,0),motor1_pos(0,1),motor1_pos(0,2)));

    // motor2 Transform
    Qt3DCore::QTransform *motor2Transform = new Qt3DCore::QTransform();
    motor2Transform->setScale(1.0f);
    motor2Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor2Transform->setTranslation(QVector3D(motor2_pos(0,0),motor2_pos(0,1),motor2_pos(0,2)));

    // motor3 Transform
    Qt3DCore::QTransform *motor3Transform = new Qt3DCore::QTransform();
    motor3Transform->setScale(1.0f);
    motor3Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor3Transform->setTranslation(QVector3D(motor3_pos(0,0),motor3_pos(0,1),motor3_pos(0,2)));

    // motor4 Transform
    Qt3DCore::QTransform *motor4Transform = new Qt3DCore::QTransform();
    motor4Transform->setScale(1.0f);
    motor4Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    motor4Transform->setTranslation(QVector3D(motor4_pos(0,0),motor4_pos(0,1),motor4_pos(0,2)));

    // arm1 Transform
    Qt3DCore::QTransform *arm1Transform = new Qt3DCore::QTransform();
    arm1Transform->setScale(1.0f);
    arm1Transform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2)),QVector3D(b3(0,0),b3(0,1),b3(0,2))));
    arm1Transform->setTranslation(QVector3D(pos(0,0),pos(0,1),pos(0,2)));

    // arm2 Transform
    Qt3DCore::QTransform *arm2Transform = new Qt3DCore::QTransform();
    arm2Transform->setScale(1.0f);
    arm2Transform->setRotation(QQuaternion::fromAxes(QVector3D(b2(0,0),b2(0,1),b2(0,2)),QVector3D(-b1(0,0),-b1(0,1),-b1(0,2)),QVector3D(b3(0,0),b3(0,1),b3(0,2))));
    arm2Transform->setTranslation(QVector3D(pos(0,0),pos(0,1),pos(0,2)));

    // up Transform
    Qt3DCore::QTransform *upTransform = new Qt3DCore::QTransform();
    upTransform->setScale(1.0f);
    upTransform->setRotation(QQuaternion::fromAxes(QVector3D(b1(0,0),b1(0,1),b1(0,2)),QVector3D(-b3(0,0),-b3(0,1),-b3(0,2)),QVector3D(b2(0,0),b2(0,1),b2(0,2))));
    upTransform->setTranslation(QVector3D(up_pos(0,0),up_pos(0,1),up_pos(0,2)));

    quad_motor1->addComponent(motor1Transform);
    quad_motor2->addComponent(motor2Transform);
    quad_motor3->addComponent(motor3Transform);
    quad_motor4->addComponent(motor4Transform);
    quad_arm1->addComponent(arm1Transform);
    quad_arm2->addComponent(arm2Transform);
    quad_up->addComponent(upTransform);
}
