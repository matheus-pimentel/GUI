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
