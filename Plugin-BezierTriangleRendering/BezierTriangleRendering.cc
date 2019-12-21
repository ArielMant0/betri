#include "BezierTriangleRendering.hh"

namespace betri
{

void randomMeshUV(BezierTMesh &mesh)
{
	mesh.request_vertex_texcoords2D();
	for (auto f_it = mesh.faces_sbegin(); f_it != mesh.faces_end(); ++f_it) {
		float tcU = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (1.0 - 0.0)));
		float tcV = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (1.0 - 0.0)));
		float angle = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * M_PI - 0.0)));

		auto fv_it = mesh.fv_iter(*f_it);
		auto vh0 = *fv_it++;
		auto vh1 = *fv_it++;
		auto vh2 = *fv_it;

		// TODO shorten this method

		// Fill all three if they are empty
		if (mesh.texcoord2D(vh0)[0] < -1000.0 && mesh.texcoord2D(vh1)[0] < -1000.0 && mesh.texcoord2D(vh2)[0] < -1000.0) {
			//BezierTMesh::TexCoord2D tc(tcU, tcV);
			BezierTMesh::TexCoord2D tcZero(0.0, 0.0);
			BezierTMesh::TexCoord2D tcZDir(1.0, 0.0);
			mesh.set_texcoord2D(vh0, tcZero);

			auto vecV01 = mesh.point(vh1) - mesh.point(vh0);
			auto vecV01_n = vecV01;
			auto vecV02 = mesh.point(vh2) - mesh.point(vh0);
			auto vecV02_n = vecV02;
			auto angle = acos(dot(normalize(vecV01_n), normalize(vecV02_n)));

			mesh.set_texcoord2D(vh1, tcZDir * length(vecV01));
			auto tc = BezierTMesh::TexCoord2D(
				cos(angle) * tcZDir[0] - sin(angle) * tcZDir[1],
				sin(angle) * tcZDir[0] + cos(angle) * tcZDir[1]
			) * length(vecV02);
			mesh.set_texcoord2D(vh2, tc);
		}
		// Fill two if they are empty
		else if (mesh.texcoord2D(vh0)[0] > -1000.0 ^ mesh.texcoord2D(vh1)[0] > -1000.0 ^ mesh.texcoord2D(vh2)[0] > -1000.0) {
			continue;
			BezierTMesh::VertexHandle first;
			BezierTMesh::VertexHandle second;
			BezierTMesh::VertexHandle third;

			if (mesh.texcoord2D(vh0)[0] > -1000.0) {
				first = vh0;
				second = vh1;
				third = vh2;
			} else if (mesh.texcoord2D(vh1)[0] > -1000.0) {
				first = vh1;
				second = vh2;
				third = vh0;
			} else if (mesh.texcoord2D(vh2)[0] > -1000.0) {
				first = vh2;
				second = vh0;
				third = vh1;
			}

			BezierTMesh::TexCoord2D tcSecond(1.0, 0.0);
			mesh.set_texcoord2D(second, tcSecond + mesh.texcoord2D(first));

			auto vecV01 = mesh.point(second) - mesh.point(first);
			auto vecV01_n = vecV01;
			auto vecV02 = mesh.point(third) - mesh.point(first);
			auto vecV02_n = vecV02;
			auto angle = acos(dot(normalize(vecV01_n), normalize(vecV02_n)));

			auto tc = BezierTMesh::TexCoord2D(
				cos(angle) * tcSecond[0] - sin(angle) * tcSecond[1],
				sin(angle) * tcSecond[0] + cos(angle) * tcSecond[1]
			) * length(vecV02) + mesh.texcoord2D(first);
			mesh.set_texcoord2D(third, tc);
		}
		// Fill one if it is empty
		else if (mesh.texcoord2D(vh0)[0] < -1000.0 ^ mesh.texcoord2D(vh1)[0] < -1000.0 ^ mesh.texcoord2D(vh2)[0] < -1000.0) {

			BezierTMesh::VertexHandle first;
			BezierTMesh::VertexHandle second;
			BezierTMesh::VertexHandle third;

			if (mesh.texcoord2D(vh0)[0] < -1000.0) {
				first = vh1;
				second = vh2;
				third = vh0;
			} else if (mesh.texcoord2D(vh1)[0] < -1000.0) {
				first = vh2;
				second = vh0;
				third = vh1;
			} else if (mesh.texcoord2D(vh2)[0] < -1000.0) {
				first = vh0;
				second = vh1;
				third = vh2;
			}

			BezierTMesh::TexCoord2D tcSecond = mesh.texcoord2D(second) - mesh.texcoord2D(first);

			auto vecV01 = mesh.point(second) - mesh.point(first);
			auto vecV01_n = vecV01;
			auto vecV02 = mesh.point(third) - mesh.point(first);
			auto vecV02_n = vecV02;
			auto angle = acos(dot(normalize(vecV01_n), normalize(vecV02_n)));

			auto tc = BezierTMesh::TexCoord2D(
				cos(angle) * tcSecond[0] - sin(angle) * tcSecond[1],
				sin(angle) * tcSecond[0] + cos(angle) * tcSecond[1]
			) * length(vecV02) + mesh.texcoord2D(first);
			mesh.set_texcoord2D(third, tc);
		}
	}
}

}
