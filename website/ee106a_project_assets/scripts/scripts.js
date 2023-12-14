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

// const csv_path = '/ros_workspaces/src/head_gui/src/points.csv';
const csv_path = 'https://raw.githubusercontent.com/naichenzhao/EECS106A_Project/main/ros_workspaces/src/head_gui/src/points.csv';

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

let slideIndex = 1;
showSlides(slideIndex);

// Next/previous controls
function plusSlides(n) {
  showSlides(slideIndex += n);
}

// Thumbnail image controls
function currentSlide(n) {
  showSlides(slideIndex = n);
}

function showSlides(n) {
  let i;
  let slides = document.getElementsByClassName("carousel-card");
  let dots = document.getElementsByClassName("dot");
  if (n > slides.length) {slideIndex = 1}
  if (n < 1) {slideIndex = slides.length}
  for (i = 0; i < slides.length; i++) {
    slides[i].style.display = "none";
  }
  for (i = 0; i < dots.length; i++) {
    dots[i].className = dots[i].className.replace(" active", "");
  }
  slides[slideIndex-1].style.display = "block";
  dots[slideIndex-1].className += " active";
}