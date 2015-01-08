/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 15:47
 */
package Box2D.Collision
{
	import Box2D.Collision.Contact.b2ContactID;
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Manifold.b2ManifoldPoint;
	import Box2D.Collision.Shapes.b2EdgeShape;
	import Box2D.Collision.Shapes.b2PolygonShape;
	import Box2D.Collision.Structures.b2ClipVertex;
	import Box2D.Collision.Structures.b2EPAxis;
	import Box2D.Collision.Structures.b2ReferenceFace;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2SPoint;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * This class collides and edge and a polygon, taking into account edge adjacency.
	 */
	public class b2EPCollider
	{
		public var m_polygonB:b2TempPolygon;

		public var m_xf:b2Mat22;

		public var m_centroidBX:Number;
		public var m_centroidBY:Number;

		public var m_v0X:Number;
		public var m_v0Y:Number;
		public var m_v1X:Number;
		public var m_v1Y:Number;
		public var m_v2X:Number;
		public var m_v2Y:Number;
		public var m_v3X:Number;
		public var m_v3Y:Number;

		public var m_normalX:Number;
		public var m_normalY:Number;
		public var m_normal0X:Number;
		public var m_normal0Y:Number;
		public var m_normal1X:Number;
		public var m_normal1Y:Number;
		public var m_normal2X:Number;
		public var m_normal2Y:Number;

//		VertexType m_type1, m_type2;

		public var m_lowerLimitX:Number;
		public var m_lowerLimitY:Number;
		public var m_upperLimitX:Number;
		public var m_upperLimitY:Number;

		public var m_radius:Number;
		public var m_front:Boolean;

		//
		private var _helperPoint:b2SPoint;
		private var _helperAxis_1:b2EPAxis;
		private var _helperAxis_2:b2EPAxis;

		/**
		 */
		public function b2EPCollider()
		{
			m_polygonB = new b2TempPolygon();
			m_xf = b2Mat22.Get();
			_helperPoint = b2SPoint.Get();
			_helperAxis_1 = new b2EPAxis();
			_helperAxis_2 = new b2EPAxis();
		}

		/**
		 * Algorithm:
		 * 1. Classify v1 and v2
		 * 2. Classify polygon centroid as front or back
		 * 3. Flip normal if necessary
		 * 4. Initialize normal range to [-pi, pi] about face normal
		 * 5. Adjust normal range according to adjacent edges
		 * 6. Visit each separating axes, only accept axes within the range
		 * 7. Return if _any_ axis indicates separation
		 * 8. Clip
		 *
		 *
		 * @param p_manifold
		 * @param p_edgeA
		 * @param p_xfA
		 * @param p_polygonB
		 * @param p_xfB
		 */
		public function Collide(p_manifold:b2Manifold, p_edgeA:b2EdgeShape, p_xfA:b2Mat22,
		                        p_polygonB:b2PolygonShape, p_xfB:b2Mat22):void
		{
			b2Math.MulTTrTr(p_xfA, p_xfB, m_xf);
			b2Math.MulTrV(m_xf, p_polygonB.m_centroidX, p_polygonB.m_centroidY, _helperPoint);

			m_centroidBX = _helperPoint.x;
			m_centroidBY = _helperPoint.y;

			m_v0X = p_edgeA.m_vertex0X;
			m_v0Y = p_edgeA.m_vertex0Y;
			m_v1X = p_edgeA.m_vertex1X;
			m_v1Y = p_edgeA.m_vertex1Y;
			m_v2X = p_edgeA.m_vertex2X;
			m_v2Y = p_edgeA.m_vertex2Y;
			m_v3X = p_edgeA.m_vertex3X;
			m_v3Y = p_edgeA.m_vertex3Y;

			var hasVertex0:Boolean = p_edgeA.m_hasVertex0;
			var hasVertex3:Boolean = p_edgeA.m_hasVertex3;

			var edge1X:Number = m_v2X - m_v1X;
			var edge1Y:Number = m_v2Y - m_v1Y;

			// normalization edge1
			var invLength:Number = b2Math.invLength(edge1X, edge1Y);
			edge1X *= invLength;
			edge1Y *= invLength;

			//
			m_normal1X = edge1Y;
			m_normal1Y = -edge1X;

			var offset0:Number = 0;
			var offset1:Number = b2Math.Dot(m_normal1X, m_normal1Y, (m_centroidBX - m_v1X), (m_centroidBY - m_v1Y));
			var offset2:Number = 0;
			var convex1:Boolean = false;
			var convex2:Boolean = false;

			// Is there a preceding edge?
			if (hasVertex0)
			{
				var edge0X:Number = m_v1X - m_v0X;
				var edge0Y:Number = m_v1Y - m_v0Y;

				invLength = b2Math.invLength(edge0X, edge0Y);
				edge0X *= invLength;
				edge0Y *= invLength;

				m_normal0X = edge0Y;
				m_normal0Y = -edge0X;

				convex1 = b2Math.CrossVectors(edge0X, edge0Y, edge1X, edge1Y) >= 0.0;
				offset0 = b2Math.Dot(m_normal0X, m_normal0Y, (m_centroidBX - m_v0X), (m_centroidBY - m_v0Y));
			}

			// Is there a following edge?
			if (hasVertex3)
			{
				var edge2X:Number = m_v3X - m_v2X;
				var edge2Y:Number = m_v3Y - m_v2Y;

				invLength = b2Math.invLength(edge2X, edge2Y);
				edge2X *= invLength;
				edge2Y *= invLength;

				m_normal2X = edge2Y;
				m_normal2Y = -edge2X;

				convex2 = b2Math.CrossVectors(edge1X, edge1Y, edge2X, edge2Y) > 0.0;
				offset2 = b2Math.Dot(m_normal2X, m_normal2Y, (m_centroidBX - m_v2X), (m_centroidBY - m_v2Y));
			}

			// Determine front or back collision. Determine collision normal limits.
			if (hasVertex0 && hasVertex3)
			{
				if (convex1 && convex2)
				{
					m_front = offset0 >= 0.0 || offset1 >= 0.0 || offset2 >= 0.0;

					if (m_front)
					{
						m_normalX = m_normal1X;
						m_normalY = m_normal1Y;

						m_lowerLimitX = m_normal0X;
						m_lowerLimitY = m_normal0Y;

						m_upperLimitX = m_normal2X;
						m_upperLimitY = m_normal2Y;
					}
					else
					{
						m_normalX = -m_normal1X;
						m_normalY = -m_normal1Y;

						m_lowerLimitX = -m_normal1X;
						m_lowerLimitY = -m_normal1Y;

						m_upperLimitX = -m_normal1X;
						m_upperLimitY = -m_normal1Y;
					}
				}
				else if (convex1)
				{
					m_front = offset0 >= 0.0 || (offset1 >= 0.0 && offset2 >= 0.0);
					if (m_front)
					{
						m_normalX = m_normal1X;
						m_normalY = m_normal1Y;

						m_lowerLimitX = m_normal0X;
						m_lowerLimitY = m_normal0Y;

						m_upperLimitX = m_normal1X;
						m_upperLimitY = m_normal1Y;
					}
					else
					{
						m_normalX = -m_normal1X;
						m_normalY = -m_normal1Y;

						m_lowerLimitX = -m_normal2X;
						m_lowerLimitY = -m_normal2Y;

						m_upperLimitX = -m_normal1X;
						m_upperLimitY = -m_normal1Y;

					}
				}
				else if (convex2)
				{
					m_front = offset2 >= 0.0 || (offset0 >= 0.0 && offset1 >= 0.0);

					if (m_front)
					{
						m_normalX = m_normal1X;
						m_normalY = m_normal1Y;

						m_lowerLimitX = m_normal1X;
						m_lowerLimitY = m_normal1Y;

						m_upperLimitX = m_normal2X;
						m_upperLimitY = m_normal2Y;
					}
					else
					{
						m_normalX = -m_normal1X;
						m_normalY = -m_normal1Y;

						m_lowerLimitX = -m_normal1X;
						m_lowerLimitY = -m_normal1Y;

						m_upperLimitX = -m_normal0X;
						m_upperLimitY = -m_normal0Y;
					}
				}
				else
				{
					m_front = offset0 >= 0.0 && offset1 >= 0.0 && offset2 >= 0.0;

					if (m_front)
					{
						m_normalX = m_normal1X;
						m_normalY = m_normal1Y;

						m_lowerLimitX = m_normal1X;
						m_lowerLimitY = m_normal1Y;

						m_upperLimitX = m_normal1X;
						m_upperLimitY = m_normal1Y;
					}
					else
					{
						m_normalX = -m_normal1X;
						m_normalY = -m_normal1Y;

						m_lowerLimitX = -m_normal2X;
						m_lowerLimitY = -m_normal2Y;

						m_upperLimitX = -m_normal0X;
						m_upperLimitY = -m_normal0Y;
					}
				}
			}
			else if (hasVertex0)
			{
				if (convex1)
				{
					m_front = offset0 >= 0.0 || offset1 >= 0.0;

					if (m_front)
					{
						m_normalX = m_normal1X;
						m_normalY = m_normal1Y;

						m_lowerLimitX = m_normal0X;
						m_lowerLimitY = m_normal0Y;

						m_upperLimitX = -m_normal1X;
						m_upperLimitY = -m_normal1Y;
					}
					else
					{
						m_normalX = -m_normal1X;
						m_normalY = -m_normal1Y;

						m_lowerLimitX = m_normal1X;
						m_lowerLimitY = m_normal1Y;

						m_upperLimitX = -m_normal1X;
						m_upperLimitY = -m_normal1Y;
					}
				}
				else
				{
					m_front = offset0 >= 0.0 && offset1 >= 0.0;

					if (m_front)
					{
						m_normalX = m_normal1X;
						m_normalY = m_normal1Y;

						m_lowerLimitX = m_normal1X;
						m_lowerLimitY = m_normal1Y;

						m_upperLimitX = -m_normal1X;
						m_upperLimitY = -m_normal1Y;
					}
					else
					{
						m_normalX = -m_normal1X;
						m_normalY = -m_normal1Y;

						m_lowerLimitX = m_normal1X;
						m_lowerLimitY = m_normal1Y;

						m_upperLimitX = -m_normal0X;
						m_upperLimitY = -m_normal0Y;
					}
				}

			}
			else if (hasVertex3)
			{
				if (convex2)
				{
					m_front = offset1 >= 0.0 || offset2 >= 0.0;

					if (m_front)
					{
						m_normalX = m_normal1X;
						m_normalY = m_normal1Y;

						m_lowerLimitX = -m_normal1X;
						m_lowerLimitY = -m_normal1Y;

						m_upperLimitX = m_normal2X;
						m_upperLimitY = m_normal2Y;
					}
					else
					{
						m_normalX = -m_normal1X;
						m_normalY = -m_normal1Y;

						m_lowerLimitX = -m_normal1X;
						m_lowerLimitY = -m_normal1Y;

						m_upperLimitX = m_normal1X;
						m_upperLimitY = m_normal1Y;
					}
				}
				else
				{
					m_front = offset1 >= 0.0 && offset2 >= 0.0;

					if (m_front)
					{
						m_normalX = m_normal1X;
						m_normalY = m_normal1Y;

						m_lowerLimitX = -m_normal1X;
						m_lowerLimitY = -m_normal1Y;

						m_upperLimitX = m_normal1X;
						m_upperLimitY = m_normal1Y;
					}
					else
					{
						m_normalX = -m_normal1X;
						m_normalY = -m_normal1Y;

						m_lowerLimitX = -m_normal2X;
						m_lowerLimitY = -m_normal2Y;

						m_upperLimitX = m_normal1X;
						m_upperLimitY = m_normal1Y;
					}
				}
			}
			else
			{
				m_front = offset1 >= 0.0;

				if (m_front)
				{
					m_normalX = m_normal1X;
					m_normalY = m_normal1Y;

					m_lowerLimitX = -m_normal1X;
					m_lowerLimitY = -m_normal1Y;

					m_upperLimitX = -m_normal1X;
					m_upperLimitY = -m_normal1Y;
				}
				else
				{
					m_normalX = -m_normal1X;
					m_normalY = -m_normal1Y;

					m_lowerLimitX = m_normal1X;
					m_lowerLimitY = m_normal1Y;

					m_upperLimitX = m_normal1X;
					m_upperLimitY = m_normal1Y;
				}
			}

			// Get polygonB in frameA
			var count:int = p_polygonB.m_count;
			var pbVertices:Vector.<Number> = p_polygonB.m_vertices;
			var pbNormals:Vector.<Number> = p_polygonB.m_normals;
			var mbVertices:Vector.<Number> = m_polygonB.vertices;
			var mbNormals:Vector.<Number> = m_polygonB.normals;
			var tX:Number;
			var tY:Number;
			m_polygonB.count = count;

			for (var i:int = 0; i < count; i++)
			{
				// vertices
				tX = b2Math.getX(pbVertices, i);
				tY = b2Math.getY(pbVertices, i);

				b2Math.MulTrV(m_xf, tX, tY, _helperPoint);
				b2Math.setXY(_helperPoint.x, _helperPoint.y, mbVertices, i);

				// normals
				tX = b2Math.getX(pbNormals, i);
				tY = b2Math.getY(pbNormals, i);

				b2Math.MulRV(m_xf, tX, tY, _helperPoint);
				b2Math.setXY(_helperPoint.x, _helperPoint.y, mbNormals, i);
			}

			m_radius = 2.0 * b2Settings.polygonRadius;

			p_manifold.pointCount = 0;

			var edgeAxis:b2EPAxis = ComputeEdgeSeparation(_helperAxis_1);

			// If no valid normal can be found than this edge should not collide.
			if (edgeAxis.type == b2EPAxis.e_unknown || edgeAxis.separation > m_radius)
			{
				return;
			}

			var polygonAxis:b2EPAxis = ComputePolygonSeparation(_helperAxis_2);
			if (polygonAxis.type != b2EPAxis.e_unknown && polygonAxis.separation > m_radius)
			{
				return;
			}

			// Use hysteresis for jitter reduction.
			var k_relativeTol:Number = 0.98;
			var k_absoluteTol:Number = 0.001;

			var primaryAxis:b2EPAxis;
			if (polygonAxis.type == b2EPAxis.e_unknown || !(polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol))
			{
				primaryAxis = edgeAxis;
			}
			else
			{
				primaryAxis = polygonAxis;
			}

			//
			var isPrimaryAxisTypeEdgeA:Boolean = primaryAxis.type == b2EPAxis.e_edgeA;
			var primaryAxisIndex:int = primaryAxis.index;

			//
			var ie:Vector.<b2ClipVertex> = new <b2ClipVertex>[b2ClipVertex.Get(), b2ClipVertex.Get()];
			var rf:b2ReferenceFace = new b2ReferenceFace();
			var cv:b2ClipVertex;
			var contactId:b2ContactID;

			if (isPrimaryAxisTypeEdgeA)
			{
				p_manifold.type = b2Manifold.FACE_A;

				// Search for the polygon normal that is most anti-parallel to the edge normal.
				var bestIndex:int = 0;
				tX = b2Math.getX(mbNormals, 0);
				tY = b2Math.getY(mbNormals, 0);

				var bestValue:Number = b2Math.Dot(m_normalX, m_normalY, tX, tY);
				var currentValue:Number;

				for (i = 1; i < count; i++)
				{
					tX = b2Math.getX(mbNormals, i);
					tY = b2Math.getY(mbNormals, i);

					currentValue = b2Math.Dot(m_normalX, m_normalY, tX, tY);

					if (currentValue < bestValue)
					{
						bestValue = currentValue;
						bestIndex = i;
					}
				}

				var i1:int = bestIndex;
				var i2:int = (i1 + 1) < count ? i1 + 1 : 0;

				//
				cv = ie[0];
				cv.vX = b2Math.getX(mbVertices, i1);
				cv.vY = b2Math.getY(mbVertices, i1);

				contactId = cv.id;
				contactId.indexA = 0;
				contactId.indexB = i1;
				contactId.typeA = b2ContactID.FACE_CF_TYPE;
				contactId.typeB = b2ContactID.VERTEX_CF_TYPE;

				//
				cv = ie[1];
				cv.vX = b2Math.getX(mbVertices, i2);
				cv.vY = b2Math.getY(mbVertices, i2);

				contactId = cv.id;
				contactId.indexA = 0;
				contactId.indexB = i2;
				contactId.typeA = b2ContactID.FACE_CF_TYPE;
				contactId.typeB = b2ContactID.VERTEX_CF_TYPE;

				//
				if (m_front)
				{
					rf.i1 = 0;
					rf.i2 = 1;
					rf.v1X = m_v1X;
					rf.v1Y = m_v1Y;
					rf.v2X = m_v2X;
					rf.v2Y = m_v2Y;
					rf.normalX = m_normal1X;
					rf.normalY = m_normal1Y;
				}
				else
				{
					rf.i1 = 1;
					rf.i2 = 0;
					rf.v1X = m_v2X;
					rf.v1Y = m_v2Y;
					rf.v2X = m_v1X;
					rf.v2Y = m_v1Y;
					rf.normalX = -m_normal1X;
					rf.normalY = -m_normal1Y;
				}
			}
			else
			{
				p_manifold.type = b2Manifold.FACE_B;

				//
				cv = ie[0];
				cv.vX = m_v1X;
				cv.vY = m_v1Y;

				contactId = cv.id;
				contactId.indexA = 0;
				contactId.indexB = primaryAxisIndex;
				contactId.typeA = b2ContactID.VERTEX_CF_TYPE;
				contactId.typeB = b2ContactID.FACE_CF_TYPE;

				//
				cv = ie[1];
				cv.vX = m_v2X;
				cv.vY = m_v2Y;

				contactId = cv.id;
				contactId.indexA = 0;
				contactId.indexB = primaryAxisIndex;
				contactId.typeA = b2ContactID.VERTEX_CF_TYPE;
				contactId.typeB = b2ContactID.FACE_CF_TYPE;

				rf.i1 = primaryAxisIndex;
				rf.i2 = (rf.i1 + 1) < count ? rf.i1 + 1 : 0;

				rf.v1X = b2Math.getX(mbVertices, rf.i1);
				rf.v1Y = b2Math.getY(mbVertices, rf.i1);

				rf.v2X = b2Math.getX(mbVertices, rf.i2);
				rf.v2Y = b2Math.getY(mbVertices, rf.i2);

				rf.normalX = b2Math.getX(mbNormals, rf.i1);
				rf.normalY = b2Math.getY(mbNormals, rf.i1);
			}

			rf.sideNormal1X = rf.normalY;
			rf.sideNormal1Y = -rf.normalX;

			rf.sideNormal2X = -rf.sideNormal1X;
			rf.sideNormal2Y = -rf.sideNormal1Y;

			rf.sideOffset1 = b2Math.Dot(rf.sideNormal1X, rf.sideNormal1Y, rf.v1X, rf.v1Y);
			rf.sideOffset2 = b2Math.Dot(rf.sideNormal2X, rf.sideNormal2Y, rf.v2X, rf.v2Y);

			// Clip incident edge against extruded edge1 side edges.
			var clipPoints1:Vector.<b2ClipVertex> = new <b2ClipVertex>[b2ClipVertex.Get(), b2ClipVertex.Get()];
			var clipPoints2:Vector.<b2ClipVertex> = ie;
			var np:int;

			// Clip to box side 1
			np = b2Collision.b2ClipSegmentToLine(clipPoints1, ie, rf.sideNormal1X, rf.sideNormal1Y, rf.sideOffset1, rf.i1);

			if (np < b2Settings.maxManifoldPoints)
			{
				return;
			}

			// Clip to negative box side 1
			np = b2Collision.b2ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2X, rf.sideNormal2Y, rf.sideOffset2, rf.i2);

			if (np < b2Settings.maxManifoldPoints)
			{
				return;
			}

			// Now clipPoints2 contains the clipped points.
			if (isPrimaryAxisTypeEdgeA)
			{
				p_manifold.localNormalX = rf.normalX;
				p_manifold.localNormalY = rf.normalY;

				p_manifold.localPointX = rf.v1X;
				p_manifold.localPointY = rf.v1Y;
			}
			else
			{
				p_manifold.localNormalX = b2Math.getX(pbNormals, rf.i1);
				p_manifold.localNormalY = b2Math.getY(pbNormals, rf.i1);

				p_manifold.localPointX = b2Math.getX(pbVertices, rf.i1);
				p_manifold.localPointY = b2Math.getY(pbVertices, rf.i1);
			}

			var pointCount:int = 0;
			var separation:Number;
			var tCV:b2ClipVertex;
			var tID:b2ContactID;
			var cp:b2ManifoldPoint;
			var cpID:b2ContactID;
			var manifoldPoints:Vector.<b2ManifoldPoint> = p_manifold.points;

			for (i = 0; i < b2Settings.maxManifoldPoints; i++)
			{
				tCV = clipPoints2[i];
				separation = b2Math.Dot(rf.normalX, rf.normalY, (tCV.vX - rf.v1X), (tCV.vY - rf.v1Y));

				if (separation <= m_radius)
				{
					tID = tCV.id;
					cp = manifoldPoints[pointCount];
					cpID = cp.id;

					if (isPrimaryAxisTypeEdgeA)
					{
						b2Math.MulTTrV(m_xf, tCV.vX, tCV.vY, _helperPoint);
						cp.localPointX = _helperPoint.x;
						cp.localPointY = _helperPoint.y;
						cpID.Set(tID);
					}
					else
					{
						cp.localPointX = tCV.vX;
						cp.localPointY = tCV.vY;
						cpID.typeA = tID.typeB;
						cpID.typeB = tID.typeA;
						cpID.indexA = tID.indexB;
						cpID.indexB = tID.indexA;
					}

					++pointCount;
				}
			}

			p_manifold.pointCount = pointCount;
		}

		/**
		 * Writes result of computation to p_axis.
		 * If p_axis is not given creates and returns new instance of b2EPAxis.
		 */
		[Inline]
		final private function ComputeEdgeSeparation(p_axis:b2EPAxis = null):b2EPAxis
		{
			var axis:b2EPAxis = p_axis ? p_axis : new b2EPAxis();
			axis.type = b2EPAxis.e_edgeA;
			axis.index = m_front ? 0 : 1;

			var bestSeparation:Number = Number.MAX_VALUE;
			var vertices:Vector.<Number> = m_polygonB.vertices;
			var v1X:Number = m_v1X;
			var v1Y:Number = m_v1Y;
			var normalX:Number = m_normalX;
			var normalY:Number = m_normalY;
			var vX:Number;
			var vY:Number;
			var s:Number;
			var count:int = m_polygonB.count;

			for (var i:int = 0; i < count; i++)
			{
				vX = b2Math.getX(vertices, i) - v1X;
				vY = b2Math.getY(vertices, i) - v1Y;

				s = normalX * vX + normalY * vY;

				if (s < bestSeparation)
				{
					bestSeparation = s;
				}
			}

			axis.separation = bestSeparation;

			return axis;
		}

		/**
		 * Writes result of computation to p_axis.
		 * If p_axis is not given creates and returns new instance of b2EPAxis.
		 */
		[Inline]
		final private function ComputePolygonSeparation(p_axis:b2EPAxis = null):b2EPAxis
		{
			var axis:b2EPAxis = p_axis ? p_axis : new b2EPAxis();
			axis.type = b2EPAxis.e_unknown;
			axis.index = -1;
			axis.separation = -Number.MAX_VALUE;

			var perpX:Number = -m_normalY;
			var perpY:Number = -m_normalX;

			var count:int = m_polygonB.count;
			var nX:Number;
			var nY:Number;
			var vX:Number;
			var vY:Number;
			var normals:Vector.<Number>;
			var vertices:Vector.<Number>;

			for (var i:int = 0; i < count; i++)
			{
				normals = m_polygonB.normals;
				nX = -b2Math.getX(normals, i);
				nY = -b2Math.getY(normals, i);

				vertices = m_polygonB.vertices;
				vX = b2Math.getX(vertices, i);
				vY = b2Math.getY(vertices, i);

				var s1:Number = nX * (vX - m_v1X) + nY * (vY - m_v1Y);
				var s2:Number = nX * (vX - m_v2X) + nY * (vY - m_v2Y);
				var s:Number = b2Math.Min(s1, s2);

				if (s > m_radius)
				{
					// No collision
					axis.type = b2EPAxis.e_edgeB;
					axis.index = i;
					axis.separation = s;
					return axis;
				}

				// Adjacency
				if ((nX * perpX + nY * perpY) >= 0.0)
				{
					vX = nX - m_upperLimitX;
					vY = nY - m_upperLimitY;
				}
				else
				{
					vX = nX - m_lowerLimitX;
					vY = nY - m_lowerLimitY;
				}

				if ((vX * m_normalX + vY * m_normalY) < -b2Settings.angularSlop)
				{
					continue;
				}

				if (s > axis.separation)
				{
					axis.type = b2EPAxis.e_edgeB;
					axis.index = i;
					axis.separation = s;
				}
			}

			return axis;
		}
	}
}
