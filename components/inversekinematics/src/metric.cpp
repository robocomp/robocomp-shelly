#include "metric.h"
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Default constructor
 */ 
Metric::Metric()
{

}
/**
 * \brief Default Desctructor.
 */ 
Metric::~Metric()
{

}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Static method that is in charge of doing the conversion os innermodel from 
 * millimeters to meters.
 * @param node the innermodel node from we walk all the kinematic tree.
 */ 
void Metric::moveInnerModelFromMillimetersToMeters(InnerModelNode* node)
{
	const  float FACTOR = 1000.f;
	FCLModelPtr fclMesh;

	InnerModelMesh 		*mesh;
	InnerModelPlane 	*plane;
	InnerModelTransform *transformation;
	InnerModelJoint 	*joint;
	
	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))   /// WE DO A CASTING OF THE NODE: TRANSFORM
	{
		if( (joint = dynamic_cast<InnerModelJoint *>(node)) == false)
		{
			transformation->setTr(transformation->getTr() / FACTOR);
			qDebug() << transformation->id << transformation->getTr();
		}
		/// WE WALK RECURSIVELY THE WHOLE KINEMATIC TREE
		for(int i=0; i<node->children.size(); i++)
			moveInnerModelFromMillimetersToMeters(node->children[i]);
	}
	else if ((plane = dynamic_cast<InnerModelPlane *>(node))) 			/// CASTING
	{
		plane->point = plane->point / FACTOR;
		plane->width /= FACTOR;
		plane->height /= FACTOR;
		plane->depth /= FACTOR;
		//printf("%s --------------------> %f %f %f\n", plane->id.toStdString().c_str(), plane->width, plane->height, plane->depth);
		plane->collisionObject->computeAABB();
		fcl::AABB a1 = plane->collisionObject->getAABB();
		fcl::Vec3f v1 = a1.center();

		// SCALE FACTOR
		{
			std::vector<fcl::Vec3f> vertices;
			vertices.push_back(fcl::Vec3f(-plane->width/2., +plane->height/2., -plane->depth/2.)); // Front NW
			vertices.push_back(fcl::Vec3f(+plane->width/2., +plane->height/2., -plane->depth/2.)); // Front NE
			vertices.push_back(fcl::Vec3f(-plane->width/2., -plane->height/2., -plane->depth/2.)); // Front SW
			vertices.push_back(fcl::Vec3f(+plane->width/2., -plane->height/2., -plane->depth/2.)); // Front SE
			vertices.push_back(fcl::Vec3f(-plane->width/2., +plane->height/2., +plane->depth/2.)); // Back NW
			vertices.push_back(fcl::Vec3f(+plane->width/2., +plane->height/2., +plane->depth/2.)); // Back NE
			vertices.push_back(fcl::Vec3f(-plane->width/2., -plane->height/2., +plane->depth/2.)); // Back SW
			vertices.push_back(fcl::Vec3f(+plane->width/2., -plane->height/2., +plane->depth/2.)); // Back SE

			osg::Matrix r;
			r.makeRotate(osg::Vec3(0, 0, 1), osg::Vec3(plane->normal(0), plane->normal(1), -plane->normal(2)));
			QMat qmatmat(4,4);
			for (int rro=0; rro<4; rro++)
				for (int cco=0; cco<4; cco++)
					qmatmat(rro,cco) = r(rro,cco);
			for (size_t i=0; i<vertices.size(); i++)
			{
				fcl::Vec3f v = vertices[i];
				const QVec rotated = (qmatmat*(QVec::vec3(v[0], v[1], v[2]).toHomogeneousCoordinates())).fromHomogeneousCoordinates();
				vertices[i] = fcl::Vec3f(rotated(0)+plane->point(0), rotated(1)+plane->point(1), rotated(2)+plane->point(2));
			}

			std::vector<fcl::Triangle> triangles;
			triangles.push_back(fcl::Triangle(0,1,2)); // Front
			triangles.push_back(fcl::Triangle(1,2,3));
			triangles.push_back(fcl::Triangle(4,5,6)); // Back
			triangles.push_back(fcl::Triangle(5,6,7));
			triangles.push_back(fcl::Triangle(4,0,6)); // Left
			triangles.push_back(fcl::Triangle(0,6,2));
			triangles.push_back(fcl::Triangle(5,1,7)); // Right
			triangles.push_back(fcl::Triangle(1,7,3));
			triangles.push_back(fcl::Triangle(5,1,4)); // Top
			triangles.push_back(fcl::Triangle(1,4,0));
			triangles.push_back(fcl::Triangle(2,3,6)); // Bottom
			triangles.push_back(fcl::Triangle(3,6,7));

			fclMesh = FCLModelPtr(new FCLModel());
			fclMesh->beginModel();
			fclMesh->addSubModel(vertices, triangles);
			fclMesh->endModel();
			plane->collisionObject = new fcl::CollisionObject(fclMesh);
		}
		plane->collisionObject->computeAABB();
		a1 = plane->collisionObject->getAABB();
		v1 = a1.center();
	}

	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)))
	{
		mesh->tx /= FACTOR; mesh->ty /= FACTOR; mesh->tz /= FACTOR;
		mesh->scalex /= FACTOR; mesh->scaley /= FACTOR; mesh->scalez /= FACTOR;
		// SCALE FACTOR
		{
			osg::Node *osgnode_ = osgDB::readNodeFile(mesh->meshPath.toStdString());
			if (not osgnode_)
			{
				printf("Could not open: '%s'.\n", mesh->meshPath.toStdString().c_str());
			}
			else
			{
				// Instanciate the vector of vertices and triangles (that's what we are looking for)
				std::vector<fcl::Vec3f> vertices;
				std::vector<fcl::Triangle> triangles;
				CalculateTriangles calcTriangles(&vertices, &triangles);
				osgnode_->accept(calcTriangles);

				// Get the internal transformation matrix of the mesh
				RTMat rtm(mesh->rx, mesh->ry, mesh->rz, mesh->tx, mesh->ty, mesh->tz);
				// Transform each of the read vertices
				for (size_t i=0; i<vertices.size(); i++)
				{
					fcl::Vec3f v = vertices[i];
					const QMat v2 = (rtm * QVec::vec3(v[0]*mesh->scalex, v[1]*mesh->scaley, -v[2]*mesh->scalez).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
					vertices[i] = fcl::Vec3f(v2(0), v2(1), v2(2));
				}
				// Associate the read vertices and triangles vectors to the FCL collision model object
				fclMesh = FCLModelPtr(new FCLModel());
				fclMesh->beginModel();
				fclMesh->addSubModel(vertices, triangles);
				fclMesh->endModel();
				mesh->collisionObject = new fcl::CollisionObject(fclMesh);
			}
		}
	}
}