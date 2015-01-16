/**
 * User: VirtualMaestro
 * Date: 14.01.2015
 * Time: 21:05
 */
package Box2D.Debug
{
	import Box2D.Collision.Shapes.b2CircleShape;
	import Box2D.Collision.Shapes.b2PolygonShape;
	import Box2D.Collision.Shapes.b2Shape;
	import Box2D.Collision.b2AABB;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2SPoint;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.b2Body;
	import Box2D.Dynamics.b2Fixture;
	import Box2D.Dynamics.b2World;

	import flash.display.Bitmap;
	import flash.display.Sprite;
	import flash.geom.Rectangle;

	use namespace b2internal;

	/**
	 * Draw debug info.
	 */
	public class DebugDraw extends Sprite
	{
		/** Constants of color */
		static public const SKY:uint = 0xff73bdd5;
		static public const BLUE:uint = 0xff0000ff;
		static public const GRASS:uint = 0xff9abe5e;
		static public const POISON:uint = 0xff85D118;
		static public const GREEN:uint = 0xff00ff00;
		static public const BLOOD:uint = 0xffe7402d;
		static public const RED:uint = 0xffff0000;
		static public const BLACK:uint = 0xff000000;
		static public const WHITE:uint = 0xffffffff;
		static public const YELLOW:uint = 0xffffff00;

		static private var _currentFlag:uint = 1;

		//
		static public const DRAW_AABB:uint = GetNextFlag();
		static public const DRAW_SHAPES:uint = GetNextFlag();

		//
		public var staticColor:uint;
		public var dynamicColor:uint;
		public var boundingColor:uint;
		public var backgroundColor:uint;
		public var contactColor:uint;

		//
		private var _canvas:Raster;
		private var _rectCanvas:Rectangle;
		private var _screenWidth:int;
		private var _screenHeight:int;
		private var _halfWidthViewport:Number;
		private var _halfHeightViewport:Number;

		public var m_world:b2World;

		// camera props
		private var _cameraWorldX:Number;
		private var _cameraWorldY:Number;
		private var _cameraViewportX:Number;
		private var _cameraViewportY:Number;
		private var _upLeftCamWorldX:Number;
		private var _upLeftCamWorldY:Number;
		private var _botRightCamWorldX:Number;
		private var _botRightCamWorldY:Number;

		//
		private var _drawFlags:uint = 0x00000000;


		// helpers
		private var _aabbHelper:b2AABB;
		private var _rectHelper:Rectangle;
		private var _pointHelper:b2SPoint;

		/**
		 */
		public function DebugDraw(p_width:int, p_height:int)
		{
			_screenWidth = p_width;
			_screenHeight = p_height;
			_halfWidthViewport = p_width * 0.5;
			_halfHeightViewport = p_height * 0.5;
			_cameraViewportX = _halfWidthViewport;
			_cameraViewportY = _halfHeightViewport;

			_canvas = new Raster(p_width, p_height, true, 0x00000000);
			var _drawHolder:Bitmap = new Bitmap(_canvas, "auto", true);
			addChild(_drawHolder);

			backgroundColor = BLACK;
			staticColor = SKY;
			dynamicColor = GRASS;
			boundingColor = WHITE;
			contactColor = BLOOD;

			setCameraPosition(0, 0);
			setDrawFlags(DRAW_AABB, DRAW_SHAPES);

			_rectCanvas = new Rectangle(0, 0, p_width, p_height);
			_aabbHelper = b2AABB.Get();
			_rectHelper = new Rectangle();
			_pointHelper = b2SPoint.Get();
		}

		/**
		 * Draws body.
		 */
		public function drawBody(p_body:b2Body):void
		{
			p_body.GetAABB(_aabbHelper);

			if (isIntersect(_aabbHelper.lowerBoundX, _aabbHelper.lowerBoundY, _aabbHelper.upperBoundX, _aabbHelper.upperBoundY,
						_upLeftCamWorldX, _upLeftCamWorldY, _botRightCamWorldX, _botRightCamWorldY))
			{
				//
				if ((_drawFlags & DRAW_AABB) != 0)
				{
					var dx:Number = _aabbHelper.lowerBoundX - _cameraWorldX;
					var dy:Number = _aabbHelper.lowerBoundY - _cameraWorldY;
					var widthAABB:Number = _aabbHelper.upperBoundX - _aabbHelper.lowerBoundX;
					var heightAABB:Number = _aabbHelper.upperBoundY - _aabbHelper.lowerBoundY;

					_rectHelper.x = _cameraViewportX + dx;
					_rectHelper.y = _cameraViewportY + dy;
					_rectHelper.width = widthAABB;
					_rectHelper.height = heightAABB;

					_canvas.drawRect(_rectHelper, GRASS);
				}

				//
				if ((_drawFlags & DRAW_SHAPES) != 0)
				{
					for(var fixture:b2Fixture = p_body.m_fixtureList; fixture; fixture = fixture.m_next)
					{
						drawShape(fixture.m_shape);
					}
				}
			}
		}

		/**
		 */
		[Inline]
		final private function drawShape(p_shape:b2Shape):void
		{
			var body:b2Body = p_shape.GetFixture().GetBody();

			switch (p_shape.m_type)
			{
				case b2Shape.CIRCLE:
				{
					var circle:b2CircleShape = p_shape as b2CircleShape;
					body.GetWorldPoint(circle.m_pX, circle.m_pY, _pointHelper);
					drawCircle(_pointHelper.x, _pointHelper.y, circle.m_radius);
					drawCenterMass(_pointHelper.x, _pointHelper.y);

					break;
				}

				case b2Shape.POLYGON:
				{
					var polygon:b2PolygonShape = p_shape as b2PolygonShape;

					var vertices:Vector.<Number> = polygon.m_vertices;
					var prevX:Number = b2Math.getX(vertices, 0);
					var prevY:Number = b2Math.getY(vertices, 0);
					var firstX:Number = prevX;
					var firstY:Number = prevY;
					var currentX:Number;
					var currentY:Number;
					var count:int = vertices.length / 2;

					for (var i:int = 1; i < count; i++)
					{
						currentX = b2Math.getX(vertices, i);
						currentY = b2Math.getY(vertices, i);

						drawLine(prevX, prevY, currentX, currentY);

						prevX = currentX;
						prevY = currentY;
					}

					drawLine(prevX, prevY, firstX, firstY);

					drawCenterMass(polygon.m_centroidX, polygon.m_centroidY);

					break;
				}

				case b2Shape.EDGE:
				{

					break;
				}

				case b2Shape.CHAIN:
				{

					break;
				}
			}
		}

		/**
		 */
		public function setCameraPosition(p_x:Number, p_y:Number):void
		{
			cameraX = p_x;
			cameraY = p_y;
		}

		[Inline]
		final public function drawCenterMass(p_x:Number, p_y:Number):void
		{
			getViewportPosition(p_x, p_y, _pointHelper);

			var size:Number = 2;
			var halfSize:Number = size/2;

			_rectHelper.x = _pointHelper.x - halfSize;
			_rectHelper.y = _pointHelper.y - halfSize;
			_rectHelper.right = _pointHelper.x + halfSize;
			_rectHelper.bottom = _pointHelper.y + halfSize;

			_canvas.drawRect(_rectHelper, BLOOD);
		}


		[Inline]
		final public function drawLine(p_x1:Number, p_y1:Number, p_x2:Number, p_y2:Number):void
		{
			getViewportPosition(p_x1, p_y1, _pointHelper);
			p_x1 = _pointHelper.x;
			p_y1 = _pointHelper.y;

			getViewportPosition(p_x2, p_y2, _pointHelper);
			p_x2 = _pointHelper.x;
			p_y2 = _pointHelper.y;

			_canvas.line(p_x1, p_y1, p_x2, p_y2, WHITE);
		}

		[Inline]
		final public function drawCircle(p_x:Number, p_y:Number, p_radius:Number):void
		{
			getViewportPosition(p_x, p_y, _pointHelper);

			_canvas.circle(_pointHelper.x, _pointHelper.y, p_radius, WHITE);
		}

		/**
		 */
		[Inline]
		final public function set cameraX(p_val:Number):void
		{
			_cameraWorldX = p_val;
			_upLeftCamWorldX = _cameraWorldX - _halfWidthViewport;
			_botRightCamWorldX = _cameraWorldX + _halfWidthViewport;
		}

		/**
		 */
		[Inline]
		final public function set cameraY(p_val:Number):void
		{
			_cameraWorldY = p_val;
			_upLeftCamWorldY = _cameraWorldY - _halfHeightViewport;
			_botRightCamWorldY = _cameraWorldY + _halfHeightViewport;
		}

		[Inline]
		final public function get cameraX():Number
		{
			return _cameraWorldX;
		}

		[Inline]
		final public function get cameraY():Number
		{
			return _cameraWorldY;
		}

		/**
		 * setDrawFlags(DebugDraw.DRAW_AABB, DebugDraw.DRAW_SHAPES);
		 */
		public function setDrawFlags(...p_flags):void
		{
			_drawFlags = 0x00000000;

			var len:int = p_flags.length;
			for (var i:int = 0; i < len; i++)
			{
				_drawFlags = _drawFlags | p_flags[i];
			}
		}

		/**
		 * Main draw. Draw everything.
		 * Need to invoke after every step.
		 */
		public function draw():void
		{
			_canvas.fillRect(_rectCanvas, backgroundColor);

			for(var body:b2Body = m_world.m_bodyList; body; body = body.m_next)
			{
				drawBody(body);
			}

//			cameraX += 1;
		}

		/**
		 * Convert given point from world coordinates to viewport's coordinates.
		 */
		[Inline]
		final private function getViewportPosition(p_posVertexX:Number, p_posVertexY:Number, p_outResult:b2SPoint):void
		{
			var dx:Number = p_posVertexX - _cameraWorldX;
			var dy:Number = p_posVertexY - _cameraWorldY;

			p_outResult.x = _cameraViewportX + dx;
			p_outResult.y = _cameraViewportY + dy;
		}

		[Inline]
		final private function isIntersect(p_leftTopX:Number, p_leftTopY:Number, p_rightBottomX:Number, p_rightBottomY:Number, p_leftTopX_1:Number,
		                                   p_leftTopY_1:Number, p_rightBottomX_1:Number, p_rightBottomY_1:Number):Boolean
		{
			if (p_leftTopX > p_rightBottomX_1 || p_rightBottomX < p_leftTopX_1 || p_rightBottomY < p_leftTopY_1 || p_leftTopY > p_rightBottomY_1)  return false;

			var exp:Boolean = false;

			if (p_leftTopX >= p_leftTopX_1)
			{
				if (p_leftTopX <= p_rightBottomX_1) exp = true;
			}

			if (!exp)
			{
				if (p_leftTopX_1 >= p_leftTopX)
				{
					if (!(p_leftTopX_1 <= p_rightBottomX)) return false;
				}
				else return false;
			}

			if (p_leftTopY >= p_leftTopY_1)
			{
				if (p_leftTopY <= p_rightBottomY_1) return true;
			}

			if (p_leftTopY_1 >= p_leftTopY)
			{
				if (p_leftTopY_1 <= p_rightBottomY) return true;
			}

			return false;
		}

		/**
		 */
		static public function GetNextFlag():uint
		{
			return (1 << (++_currentFlag - 1));
		}
	}
}
