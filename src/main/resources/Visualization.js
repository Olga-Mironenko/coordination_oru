class Visualization {

	constructor() {
		this.originalTranslate = { x : 0, y : 0 };
		this.originalScale = 1;
		this.canvas = document.createElement("canvas"); 
		this.canvas.id = "myCanvas";
		document.getElementsByTagName('body')[0].appendChild(this.canvas);
		this.ctx = this.canvas.getContext("2d");
		document.addEventListener('DOMContentLoaded', this.init(), false);
		this.geometries = {};
		this.geometryColors = {};
		this.geometryFilled = {};
		this.geometryExtraData = {};
		this.geometryTimeouts = {};
		this.deletedForever = [];

		this.selectedGeoms = new Set();
		
		//Default for scaling text (if not user defined)
		this.fontScale = -1;

		this.currentTextScale = this.originalScale;
		this.resizeCanvasToUserSpec();

		this.ctrlOverlay = document.getElementById("ctrl_overlay"); 
		this.overlay = document.getElementById("overlay"); 
		
		this.overlay.addEventListener('mousedown', this.processMouseDown(this), false);
		this.overlay.addEventListener('mousemove', this.processMouseMove(this), false);
		this.overlay.addEventListener('mouseup', this.processMouseUp(this), false);
		document.body.addEventListener('keydown', this.processKeyDown(this), false);

		
//		this.canvas.addEventListener('mousedown', this.processMouseDown(this), false);
//		//this.canvas.addEventListener('mousemove', this.processMouseMove(this), false);
//		//this.canvas.addEventListener('mouseup', this.processMouseUp(this), false);
//
//		//this.ctrlOverlay.addEventListener('mousedown' this.processMouseDown(this), false);
//		this.ctrlOverlay.addEventListener('mousemove', this.processMouseMove(this), false);
//		this.ctrlOverlay.addEventListener('mouseup', this.processMouseUp(this), false);


		this.ctrlOverlayText = document.createElement('div');
		this.ctrlOverlayText.id = "ctrl_overlay_text";
		this.ctrlOverlayText.innerHTML = "";
		this.ctrlOverlay.appendChild(this.ctrlOverlayText);
		
		this.overlayText = document.createElement('div');
		this.overlayText.id = "overlay_text";
		this.overlayText.innerHTML = "";
		this.overlay.appendChild(this.overlayText);

		//this.canvas.addEventListener('wheel', this.processWheel(this), false);
		this.isMouseDown = false;
		this.dragged = false;
		this.mousePos = { x : 0, y : 0 };
		this.dragDelta = { x : 0, y : 0 };
		this.scaleDelta = 1;

		// Create a custom fillText function that flips the canvas, draws the text, and then flips it back
		this.ctx.fillText = function(text, x, y) {
			this.save();       // Save the current canvas state
			this.scale(1, -1); // Flip to draw the text
			this.fillText.dummyCtx.fillText.call(this, text, x, -y); // Draw the text, invert y to get coordinate right
			this.restore();    // Restore the initial canvas state
		}
		// Create a dummy canvas context to use as a source for the original fillText function
		this.ctx.fillText.dummyCtx = document.createElement('canvas').getContext('2d');
		
		this.map = null;
		this.mapResolution = 1.0;
		this.mapOrigin = { x : 0, y : 0 };
		this.footprintSize = 1;
		
	}

	encode(input) {
		var keyStr = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";
		var output = "";
		var chr1, chr2, chr3, enc1, enc2, enc3, enc4;
		var i = 0;

		while (i < input.length) {
			chr1 = input[i++];
			chr2 = i < input.length ? input[i++] : Number.NaN; // Not sure if the index 
			chr3 = i < input.length ? input[i++] : Number.NaN; // checks are needed here

			enc1 = chr1 >> 2;
			enc2 = ((chr1 & 3) << 4) | (chr2 >> 4);
			enc3 = ((chr2 & 15) << 2) | (chr3 >> 6);
			enc4 = chr3 & 63;

			if (isNaN(chr2)) {
				enc3 = enc4 = 64;
			} else if (isNaN(chr3)) {
				enc4 = 64;
			}
			output += keyStr.charAt(enc1) + keyStr.charAt(enc2) +
			keyStr.charAt(enc3) + keyStr.charAt(enc4);
		}
		return output;
	}

	setMapMetadata(data) {
		this.mapResolution = data.resolution;
		this.mapOrigin.x = data.x;
		this.mapOrigin.y = data.y;
		this.mapAlpha = data.alpha !== undefined && data.alpha !== null ? data.mapAlpha : 0.4;
	}
	
	setInitialTransform(data) {
		this.originalScale = data.scale;
		this.originalTranslate.x = data.x;
		this.originalTranslate.y = data.y;
		this.resizeCanvasToUserSpec();
	}

	setMap(bytes) {
		var image = new Image();
		image.src = 'data:image/png;base64,'+this.encode(bytes);
		this.map = image;
	}

	ctrlOverlayOn() {
		this.ctrlOverlay.style.display = "block";
	}

	ctrlOverlayOff() {
		this.ctrlOverlay.style.display = "none";
	}
	
	overlayOn() {
		this.overlay.style.display = "block";
	}

	overlayOff() {
		this.overlay.style.display = "none";
	}
	
	setOverlayText(data) {
		this.overlayText.innerHTML = data.text;
	}

	updateCtrlOverlayText() {
		var decomp = this.matrix.decompose(true);
		var scale = decomp.scale.x;
		var transX = decomp.translate.x/scale;
		var transY = -(decomp.translate.y-this.canvas.height)/scale;
		
		this.ctrlOverlayText.innerHTML = "Translate (x,y): (" + transX.toFixed(2) + "," + transY.toFixed(2) + ") Scale: " + scale.toFixed(2);
	}

	processMouseDown(viz) {
		return function(e) {
			viz.isMouseDown = true;
			viz.mousePos.x = e.clientX;
			viz.mousePos.y = e.clientY;
			viz.dragged = false;
//			viz.ctrlOverlayOn();
//			viz.updateCtrlOverlayText();
		};
	}

	processMouseUp(viz) {
		return function(e) {
			viz.isMouseDown = false;
			if (viz.dragged) {
				viz.dragDelta.x = 0;
				viz.dragDelta.y = 0;
				viz.scaleDelta = 1;
				viz.ctrlOverlayOff();
			}
			else {
				var clickedKey = "";
				var tPoint = viz.matrix.inverse().applyToPoint(viz.mousePos.x, viz.mousePos.y);
				console.log("Clicked at (" + tPoint.x + "," + tPoint.y + ")");
				var message = JSON.stringify(["click", { x: tPoint.x, y: tPoint.y, theta: 0 }]);
				websocket.send(message);
				Object.keys(viz.geometries).forEach(function(key,index) {
					// key: the name of the object key
					// index: the ordinal position of the key within the object
					if (!key.startsWith("_") && !(key.includes("-"))) {
						
						//console.log(viz.clickedInside(viz.geometries[key],tPoint));
						
						if (viz.clickedInside(viz.geometries[key],tPoint)) {
							clickedKey = key;
							return;
						}

					}
				});
				viz.selectedGeoms.clear();
				viz.selectedGeoms.add(clickedKey);
			}
		};
	}
	
	processKeyDown(viz) {
		return function(e) {
			console.log("keydown: code=" + e.code);
			var message = JSON.stringify(["keydown", e.code]);
			websocket.send(message);
		};
	}
	
	clickedInside(vertices, point) {
	    // ray-casting algorithm based on
	    // https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html/pnpoly.html
	    	    
		const x = point.x
		const y = point.y

		let inside = false
		for (let i = 0, j = vertices.length - 1; i < vertices.length; j = i++) {
  			const xi = vertices[i].x,
    			yi = vertices[i].y
  				const xj = vertices[j].x,
    			yj = vertices[j].y

			const intersect = yi > y != yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi
  			if (intersect) inside = !inside
		}
		
		return inside	
	}
	
	distanceTo(coord1,coord2) {
		return Math.sqrt( (coord1.x-coord2.x)*(coord1.x-coord2.x) + (coord1.y-coord2.y)*(coord1.y-coord2.y));
	}

	processMouseMove(viz) {
		return function(e) {
			if (viz.isMouseDown) {
				viz.dragged = true;
				viz.ctrlOverlayOn();
				viz.updateCtrlOverlayText();
				////
//				var onmousestop = function() {
//					viz.dragDelta.x = 0;
//					viz.dragDelta.y = 0;
//					viz.scaleDelta = 1;
//					viz.mousePos.x = e.clientX;
//					viz.mousePos.y = e.clientY;
//				}, thread;
//				clearTimeout(thread);
//				thread = setTimeout(onmousestop, 10);
//				viz.thread = setTimeout(viz.onMouseStop, 50);
				////
				if (e.ctrlKey) {
					if (e.clientY-viz.mousePos.y < 0) viz.scaleDelta -= 0.001;
					else viz.scaleDelta += 0.001;
				}
				else {
					viz.dragDelta.x = e.clientX-viz.mousePos.x;
					viz.dragDelta.y = e.clientY-viz.mousePos.y;
				}
			}
		};
	}

	init() {
		this.resizeCanvasToDisplaySize();
	}

	addGeometry(geom) {
		if (!this.deletedForever.includes(geom.name)) {
			var timeNow = new Date().getTime();
			//if (geom.name.startsWith("_") && !(geom.name.includes("-"))) console.log(geom.name + " in geometries? " + (geom.name in this.geometries));
			this.geometries[geom.name] = geom.coordinates;
			this.geometryColors[geom.name] = geom.color;
			this.geometryFilled[geom.name] = geom.filled;
			if (geom.age != null) this.geometryTimeouts[geom.name] = timeNow+geom.age;
			if (geom.extraData != null) this.geometryExtraData[geom.name] = geom.extraData;
		}
		else {
			console.log("Ignored spurious geometry " + geom.name);
		}
	}

	removeGeometry(geomName) {
		var nameToRem = geomName.name;
		//console.log("removing geom " + nameToRem);
		delete this.geometries[nameToRem];
		delete this.geometryColors[nameToRem];
		delete this.geometryFilled[nameToRem];
		delete this.geometryTimeouts[nameToRem];
		delete this.geometryExtraData[nameToRem];
		if (nameToRem.startsWith("_") && !(nameToRem.includes("-")) && !this.deletedForever.includes(nameToRem)) this.deletedForever.push(nameToRem);
	}
	
	//Not used, but should work
	updateFontScale(sc) {
		//console.log("updating font scale " + sc.value);
		this.fontScale = sc.value;
	}
	
	setFontScale(sc) {
		//console.log("setting font scale " + sc.value);
		this.fontScale = sc.scale;
	}

	refresh() {
		//Remove old geoms
		var toRemove = [];
		var timeNow = new Date().getTime();
		var viz = this;
		Object.keys(this.geometryTimeouts).forEach(function(key,index) {
			// key: the name of the object key
			// index: the ordinal position of the key within the object
			//console.log("drawing geom " + key);
			if (viz.geometryTimeouts[key] < timeNow) {
				toRemove.push(key);
			}
		});
		for (var i = 0; i < toRemove.length; i++) {
			var obj = { "name" : toRemove[i] };
			this.removeGeometry(obj);
		}

		//Clear all
		this.clearCanvas();

		//Draw map if necessary
		if (this.map != null) {
			var mapW = this.map.width*this.mapResolution;
			var mapH = this.map.height*this.mapResolution;
			this.ctx.save();
			this.ctx.scale(1,-1);
			this.ctx.translate(0,-mapH);
			this.ctx.globalAlpha = this.mapAlpha; // values less than one make white tunnels gray
			this.ctx.drawImage(this.map,this.mapOrigin.x,this.mapOrigin.y,mapW,mapH);
			this.ctx.globalAlpha = 1.0;
			this.ctx.restore();
		}

		//Resize properly
		this.resizeCanvasToMouseMovement();

		//Find max area
		var maxArea = -1.0;
		Object.keys(this.geometries).forEach(function(key,index) {
			// key: the name of the object key
			// index: the ordinal position of the key within the object
			if (key.startsWith("V")) {
				var area = viz.calcPolygonArea(viz.geometries[key]);
				if (area > maxArea) maxArea = area;
			}
		});
		if (maxArea < 0) maxArea = 1.0;

		//Draw all geoms
		Object.keys(this.geometries).forEach(function(key,index) {
			// key: the name of the object key
			// index: the ordinal position of the key within the object
			//console.log("drawing geom " + key);
			//var area = viz.calcPolygonArea(viz.geometries[key]);
			//var linewidth = Math.sqrt(area)/70;
			var linewidth = Math.sqrt(maxArea)/30;
			//var linewidth = 0.1;
			if (viz.selectedGeoms.has(key)) viz.drawPolygon(viz.geometries[key], viz.geometryColors[key], viz.geometryFilled[key], linewidth);
			else viz.drawPolygon(viz.geometries[key], viz.geometryColors[key], !viz.geometryFilled[key], linewidth);
			//var textSize = 0.2;
			//if (key.startsWith("V")) {
			//	//textSize = Math.sqrt(area)/2;
			//	textSize = 1.3*Math.sqrt(area);
			//}
			//var coef = Math.sqrt(maxArea);
			var textSizeCoefficient = 1;
			var textSize = viz.fontScale * textSizeCoefficient;
			//console.log("scale is " + viz.fontScale);
			//console.log("coef is " + coef);
			//console.log("textSize is " + textSize);
			if (!key.startsWith("_")) {
				var text = key;
				if (viz.geometryExtraData[key] != null) {
					text += viz.geometryExtraData[key];
				}
				viz.drawText(text,viz.geometries[key][0],"#ffffff", textSize);
			}
		});
	}

	resizeCanvasToUserSpec() {
		this.matrix = Matrix.from( 1, 0, 0, 1, 0, 0 );
		this.matrix.scale(1,-1);
		this.matrix.translate(0,-this.canvas.height);

		this.matrix.translate(this.originalTranslate.x*this.originalScale, this.originalTranslate.y*this.originalScale);
		this.matrix.scale(this.originalScale,this.originalScale);		

		this.matrix.applyToContext(this.ctx);
	}

	resizeCanvasToMouseMovement() {
		this.matrix.scale(this.scaleDelta, this.scaleDelta);
		this.matrix.translate((this.dragDelta.x/100),-(this.dragDelta.y/100));
		this.matrix.applyToContext(this.ctx);		
		this.currentTextScale *= this.scaleDelta;
		this.updateCtrlOverlayText();
	}

	calcPolygonArea(coordinates) {
		var total = 0;

		for (var i = 0, l = coordinates.length; i < l; i++) {
			var addX = coordinates[i].x;
			var addY = coordinates[i == coordinates.length - 1 ? 0 : i + 1].y;
			var subX = coordinates[i == coordinates.length - 1 ? 0 : i + 1].x;
			var subY = coordinates[i].y;

			total += (addX * addY * 0.5);
			total -= (subX * subY * 0.5);
		}

		return Math.abs(total);
	}
	
	drawPolygon(coordinates, color, empty, linewidth) {
		this.ctx.globalAlpha = 0.5;
		this.ctx.fillStyle = color;
		this.ctx.strokeStyle = color;
		this.ctx.lineWidth = linewidth;
		this.ctx.beginPath();
		this.ctx.moveTo(coordinates[0].x, coordinates[0].y);
		for (var i = 1; i < coordinates.length; i++) {
			this.ctx.lineTo(coordinates[i].x, coordinates[i].y);
		}
		this.ctx.closePath();
		if (empty) this.ctx.stroke();
		else this.ctx.fill();
		this.ctx.globalAlpha = 1.0;
	}

	drawText(text, coord, color, size) {
		var scale = size * 3;
		this.ctx.font = 'italic ' + size + 'pt Times New Roman';
		//this.ctx.font = 'italic 12pt Calibri';
		//console.log(size);
		//var w = this.ctx.measureText(text).width;
		//var h = w/text.length;
		//console.log("approx h is " + h);
		this.ctx.save();
		this.ctx.scale(scale,scale);
		this.ctx.fillStyle = color;
		this.ctx.strokeStyle = color;
		this.ctx.lineWidth=0.2;
		this.ctx.fillText(text, coord.x/scale, coord.y/scale);
		this.ctx.restore();
	}

	clearCanvas() {
		this.ctx.save();
		// Use the identity matrix while clearing the canvas
		this.ctx.setTransform(1, 0, 0, 1, 0, 0);
		this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
		// Restore the transform
		this.ctx.restore();
	}

	getBounds() {
		var minX = Number.MAX_VALUE;
		var minY = Number.MAX_VALUE;
		var maxX = Number.MIN_VALUE;
		var maxY = Number.MIN_VALUE;
		Object.keys(this.geometries).forEach(function(key,index) {
			// key: the name of the object key
			// index: the ordinal position of the key within the object
			for (var i = 0; i < viz.geometries[key].length; i++) {
				if (viz.geometries[key][i].x < minX) minX = viz.geometries[key][i].x;
				if (viz.geometries[key][i].y < minY) minY = viz.geometries[key][i].y;
				if (viz.geometries[key][i].x > maxX) maxX = viz.geometries[key][i].x;
				if (viz.geometries[key][i].y > maxY) maxY = viz.geometries[key][i].y;
			}
		});
		return { minX : minX , minY : minY, maxX : maxX, maxY : maxY };
	}

	resizeCanvasToDisplaySize() {
		this.canvas.style.width = window.innerWidth + "px";
		this.canvas.style.height = window.innerHeight + "px";
		this.canvas.width = window.innerWidth;
		this.canvas.height = window.innerHeight;
		this.canvas.style.border = "none";
		//document.body.scrollTop = 0; // <-- pull the page back up to the top
		document.body.style.overflow = 'hidden'; // <-- relevant addition
	}

}
