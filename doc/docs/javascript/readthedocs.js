document.addEventListener("DOMContentLoaded", function () {
  // Trigger Read the Docs' search addon instead of Material MkDocs default
  var searchInput = document.querySelector(".md-search__input");
  if (!searchInput) {
    return;
  }
  searchInput.addEventListener("focus", function () {
    var event = new CustomEvent("readthedocs-search-show");
    document.dispatchEvent(event);
  });
});

// Use CustomEvent to generate the version selector
document.addEventListener("readthedocs-addons-data-ready", function (event) {
  var config = event.detail.data();
  var versioning =
    '<div class="md-version">' +
    '<button class="md-version__current" aria-label="Select version">' +
    config.versions.current.slug +
    "</button>" +
    '<ul class="md-version__list">' +
    config.versions.active
      .map(function (version) {
        return (
          '<li class="md-version__item">' +
          '<a href="' +
          version.urls.documentation +
          '" class="md-version__link">' +
          version.slug +
          "</a>" +
          "</li>"
        );
      })
      .join("\n") +
    "</ul>" +
    "</div>";

  // Check if we already added versions and remove them if so.
  // This happens when using the "Instant loading" feature.
  // See https://squidfunk.github.io/mkdocs-material/setup/setting-up-navigation/#instant-loading
  var currentVersions = document.querySelector(".md-version");
  if (currentVersions !== null) {
    currentVersions.remove();
  }
  var headerTopic = document.querySelector(".md-header__topic");
  if (headerTopic) {
    headerTopic.insertAdjacentHTML("beforeend", versioning);
  }
});
