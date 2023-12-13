window.addEventListener('scroll', function() {
    let fromTop = window.scrollY;
    let tocLinks = document.querySelectorAll('#toc a');

    tocLinks.forEach(function(link) {
        let section = document.querySelector(link.hash);

        if (
            section.offsetTop <= fromTop &&
            section.offsetTop + section.offsetHeight > fromTop
        ) {
            link.classList.add('active');
        } else {
            link.classList.remove('active');
        }
    });
});

const csv_path = '/ros_workspaces/src/head_gui/src/points.csv';
// const csv_path = 'https://raw.githubusercontent.com/naichenzhao/EECS106A_Project/main/ros_workspaces/src/head_gui/src/points.csv?token=GHSAT0AAAAAACBB4JT7VLYBQMCGI4OIDZUMZLZ3IRQ';

d3.csv(csv_path, function(err, rows){
function unpack(rows, key) {
	return rows.map(function(row)
	{ return row[key]; });}

var trace1 = {
	x:unpack(rows, 'Point_X'), y: unpack(rows, 'Point_Y'), z: unpack(rows, 'Point_Z'),
	mode: 'markers',
	marker: {
		size: 5,
		line: {
		color: 'rgba(217, 217, 217, 0.14)',
		width: 0.25},
		opacity: 0.8},
	type: 'scatter3d'
};

var data = [trace1];
var layout = {margin: {
	l: 0,
	r: 0,
	b: 0,
	t: 0
  }};
Plotly.newPlot('head_gui_plot', data, layout);
});

