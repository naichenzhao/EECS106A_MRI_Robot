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