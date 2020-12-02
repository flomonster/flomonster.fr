$(document).ready(function() {
  titles = $('h2, h3');
  nb_titles = titles.length;
  sections = $(".section a, .subsection a");

  $('.section a, .subsection a').on('click', function() {
    var section = $(this).attr('href');
    var scrollPoint = $(section).offset().top - 28;
    $('body,html').animate({scrollTop : scrollPoint}, 500);
    return false;
  });

  $(window)
      .scroll(function() {
        var windscroll = $(window).scrollTop();
        var active_id = "";
        titles.each(function(index) {
          if (windscroll >= $(this).offset().top - 50 &&
              (index == nb_titles - 1 ||
               windscroll < titles.get(index + 1).offsetTop - 50)) {
            active_id = '#' + $(this).attr('id');
          }
        });
        sections.each(function(index) {
          if ($(this).attr('href') == active_id) {
            $(this).addClass('active');
          } else {
            $(this).removeClass('active');
          }
        });
      })
      .scroll();
});
