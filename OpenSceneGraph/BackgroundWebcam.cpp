/* OpenSceneGraph example, osghud.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

#include <osgUtil/Optimizer>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>
#include <osg/Material>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osg/Camera>
#include <osg/RenderInfo>
#include <osg/TextureRectangle>
#include <osgDB/WriteFile>
#include <osgText/Text>

#include <opencv2/opencv.hpp>

#include <stdio.h>


osg::Camera* createHUD(osg::Image* bgImage)
{
	// create a camera to set up the projection and model view matrices, and the subgraph to draw in the HUD
	osg::Camera* camera = new osg::Camera;

	// set the projection matrix
	camera->setProjectionMatrix(osg::Matrix::ortho2D(0, 1.0f, 1.0f, 0));
	// set the view matrix
	camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	camera->setViewMatrix(osg::Matrix::identity());

	// only clear the depth buffer
	camera->setClearMask(GL_DEPTH_BUFFER_BIT);

	// draw subgraph after main camera view.
	camera->setRenderOrder(osg::Camera::NESTED_RENDER);

	// we don't want the camera to grab event focus from the viewers main camera(s).
	camera->setAllowEventFocus(false);



	// add to this camera a subgraph to render
	{

		osg::Geode* geode = new osg::Geode();

		// turn lighting off for the text and disable depth test to ensure it's always ontop.
		osg::StateSet* stateset = geode->getOrCreateStateSet();
		stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

		{
			auto backgroundImage = bgImage; 
			auto texturedQuad = osg::createTexturedQuadGeometry(
				osg::Vec3(0.f, 0.f, 0.f), 
				osg::Vec3(1.0f, 0.f, 0.f),
				osg::Vec3(0.f, 1.0f, 0.f),
				backgroundImage->s(), 
				backgroundImage->t());
			auto textureRect = new osg::TextureRectangle(backgroundImage);
			textureRect->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
			textureRect->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
			textureRect->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
			texturedQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0, textureRect, osg::StateAttribute::ON); 
			texturedQuad->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF); 
			geode->addDrawable(texturedQuad);
		} 


		{
			osg::BoundingBox bb;
			for(unsigned int i=0;i<geode->getNumDrawables();++i)
			{
				bb.expandBy(geode->getDrawable(i)->getBoundingBox());
			}

			osg::Geometry* geom = new osg::Geometry;

			osg::Vec3Array* vertices = new osg::Vec3Array;
			float depth = bb.zMin()-0.1;
			vertices->push_back(osg::Vec3(bb.xMin(),bb.yMax(),depth));
			vertices->push_back(osg::Vec3(bb.xMin(),bb.yMin(),depth));
			vertices->push_back(osg::Vec3(bb.xMax(),bb.yMin(),depth));
			vertices->push_back(osg::Vec3(bb.xMax(),bb.yMax(),depth));
			geom->setVertexArray(vertices);

			osg::Vec3Array* normals = new osg::Vec3Array;
			normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
			geom->setNormalArray(normals, osg::Array::BIND_OVERALL);

			osg::Vec4Array* colors = new osg::Vec4Array;
			colors->push_back(osg::Vec4(1.0f,1.0,0.8f,0.2f));
			geom->setColorArray(colors, osg::Array::BIND_OVERALL);

			geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));

			osg::StateSet* stateset = geom->getOrCreateStateSet();
			stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
			//stateset->setAttribute(new osg::PolygonOffset(1.0f,1.0f),osg::StateAttribute::ON);
			stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

			geode->addDrawable(geom);
		}

		camera->addChild(geode);
	}

	return camera;
}

void main()
{
	cv::VideoCapture vcap(0); 
	if(!vcap.isOpened()){
		std::cout << "FAIL!" << std::endl;
		return;
	}
	cv::Mat *frame = new cv::Mat(cv::Mat::zeros(vcap.get(CV_CAP_PROP_FRAME_HEIGHT), vcap.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC3));

	vcap >> *frame;

	osg::ref_ptr<osg::Image> backgroundImage = new osg::Image;
	backgroundImage->setImage(frame->cols, frame->rows, 3,
		GL_RGB, GL_BGR, GL_UNSIGNED_BYTE,
		(uchar*)(frame->data),
		osg::Image::AllocationMode::NO_DELETE, 1);



	// read the scene from the list of file specified commandline args.
	osg::ref_ptr<osg::Node> scene;

	// if not loaded assume no arguments passed in, try use default model instead.
	if (!scene)
		scene = osgDB::readNodeFile("dumptruck.osgt");


	if (!scene)
	{
		osg::notify(osg::NOTICE)<<"No model loaded"<<std::endl;
		return;
	}
	// construct the viewer.
	osgViewer::Viewer viewer;


	osg::ref_ptr<osg::Group> group = new osg::Group;

	// add the HUD subgraph.
	if (scene.valid()) group->addChild(scene.get());
	group->addChild(createHUD(backgroundImage));

	// set the scene to render
	viewer.setSceneData(group.get());
	viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	viewer.realize();  // set up windows and associated threads.


	while(!viewer.done())
	{
		vcap >> *frame;
		backgroundImage->dirty();

		viewer.frame();
	}
	//viewer.run();
}
