all: \
	bin/main/index.html \
	build/resources/main/index.html \
	bin/main/Visualization.js \
	build/resources/main/Visualization.js

bin/main/index.html: src/main/resources/index.html
	cp -a $< $@
	
build/resources/main/index.html: src/main/resources/index.html
	cp -a $< $@
	
bin/main/Visualization.js: src/main/resources/Visualization.js
	cp -a $< $@
	
build/resources/main/Visualization.js: src/main/resources/Visualization.js
	cp -a $< $@