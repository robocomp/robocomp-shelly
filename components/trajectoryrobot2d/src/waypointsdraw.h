/*
 * Copyright 2016 pbustos <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef WAYPOINTSDRAW_H
#define WAYPOINTSDRAW_H

#include "innerviewer.h"
#include "waypoints.h"
#include "qline2d.h"

class WaypointsDraw
{
	public:
		WaypointsDraw();
		~WaypointsDraw();
		
		bool draw(WayPoints& road, InnerViewer* viewer, const CurrentTarget& currentTarget);
	
	private:
		void clearDraw(InnerViewer *viewer);	
};

#endif // WAYPOINTSDRAW_H
