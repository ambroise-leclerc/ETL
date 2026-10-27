// Compatibility bridge for the deprecated ETLSTD namespace macro, included only by the legacy
// libstd/include/<header> wrappers. Removed in release 0.3.
#pragma once

#warning "ETLSTD is deprecated: the libstd headers declare their facilities in std. Selecting another namespace through ETLSTD no longer works."

#ifdef ETLSTD
#undef ETLSTD
#endif
#define ETLSTD std
