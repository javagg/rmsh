#ifndef RMSHC_H
#define RMSHC_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef RMSH_API
#define RMSH_API
#endif

RMSH_API int rmshInitialize(int argc, const char *const argv[], int readConfigFiles, int run, int *ierr);
RMSH_API int rmshFinalize(int *ierr);
RMSH_API int rmshClear(int *ierr);
RMSH_API int rmshOpen(const char *fileName, int *ierr);
RMSH_API int rmshMerge(const char *fileName, int *ierr);
RMSH_API int rmshWrite(const char *fileName, int *ierr);

RMSH_API int rmshOptionSetNumber(const char *name, double value, int *ierr);
RMSH_API int rmshOptionGetNumber(const char *name, double *value, int *ierr);
RMSH_API int rmshOptionSetString(const char *name, const char *value, int *ierr);
RMSH_API int rmshOptionGetString(const char *name, char **value, int *ierr);
RMSH_API int rmshOptionSetColor(const char *name, int r, int g, int b, int a, int *ierr);
RMSH_API int rmshOptionGetColor(const char *name, int *r, int *g, int *b, int *a, int *ierr);

RMSH_API int rmshLoggerStart(int *ierr);
RMSH_API int rmshLoggerStop(int *ierr);
RMSH_API int rmshLoggerGet(char ***log, size_t *log_n, int *ierr);

RMSH_API int rmshModelAdd(const char *name, int *ierr);
RMSH_API int rmshModelRemove(int *ierr);
RMSH_API int rmshModelGetCurrent(char **name, int *ierr);
RMSH_API int rmshModelSetCurrent(const char *name, int *ierr);
RMSH_API int rmshModelGetDimension(int *dim, int *ierr);
RMSH_API int rmshModelGetEntities(int **dimTags, size_t *dimTags_n, int dim, int *ierr);
RMSH_API int rmshModelGetEntityName(int dim, int tag, char **name, int *ierr);
RMSH_API int rmshModelSetEntityName(int dim, int tag, const char *name, int *ierr);
RMSH_API int rmshModelGetBoundingBox(int dim, int tag, double *xmin, double *ymin, double *zmin, double *xmax, double *ymax, double *zmax, int *ierr);
RMSH_API int rmshModelAddPhysicalGroup(int dim, const int *tags, size_t tags_n, int tag, const char *name, int *ierr);
RMSH_API int rmshModelGetPhysicalGroups(int **dimTags, size_t *dimTags_n, int dim, int *ierr);
RMSH_API int rmshModelSetPhysicalName(int dim, int tag, const char *name, int *ierr);
RMSH_API int rmshModelGetPhysicalName(int dim, int tag, char **name, int *ierr);

RMSH_API int rmshModelGeoAddPoint(double x, double y, double z, double meshSize, int tag, int *ierr);
RMSH_API int rmshModelGeoAddLine(int startTag, int endTag, int tag, int *ierr);
RMSH_API int rmshModelGeoAddCurveLoop(const int *curveTags, size_t curveTags_n, int tag, int *ierr);
RMSH_API int rmshModelGeoAddPlaneSurface(const int *wireTags, size_t wireTags_n, int tag, int *ierr);
RMSH_API int rmshModelGeoSynchronize(int *ierr);

RMSH_API int rmshModelOccAddBox(double x, double y, double z, double dx, double dy, double dz, int tag, int *ierr);
RMSH_API int rmshModelOccAddSphere(double x, double y, double z, double r, int tag, int *ierr);
RMSH_API int rmshModelOccAddCylinder(double x, double y, double z, double dx, double dy, double dz, double r, int tag, int *ierr);
RMSH_API int rmshModelOccCut(const int *objectDimTags, size_t objectDimTags_n, const int *toolDimTags, size_t toolDimTags_n, int *ierr);
RMSH_API int rmshModelOccFuse(const int *objectDimTags, size_t objectDimTags_n, const int *toolDimTags, size_t toolDimTags_n, int *ierr);
RMSH_API int rmshModelOccFragment(const int *objectDimTags, size_t objectDimTags_n, const int *toolDimTags, size_t toolDimTags_n, int *ierr);
RMSH_API int rmshModelOccSynchronize(int *ierr);

RMSH_API int rmshModelMeshSetSize(const int *dimTags, size_t dimTags_n, double size, int *ierr);
RMSH_API int rmshModelMeshGenerate(int dim, int *ierr);
RMSH_API int rmshModelMeshSetOrder(int order, int *ierr);
RMSH_API int rmshModelMeshGetNodes(size_t *nodeTags_n, size_t *coord_n, size_t *parametricCoord_n, int dim, int tag, int includeBoundary, int returnParametricCoord, int *ierr);
RMSH_API int rmshModelMeshGetElements(size_t *elementTypes_n, size_t *elementTags_n, size_t *nodeTags_n, int dim, int tag, int *ierr);
RMSH_API int rmshModelMeshClear(const int *dimTags, size_t dimTags_n, int *ierr);
RMSH_API int rmshModelMeshOptimize(const char *method, int force, int niter, const int *dimTags, size_t dimTags_n, int *ierr);
RMSH_API int rmshModelMeshRefine(int *ierr);
RMSH_API int rmshModelMeshRecombine(int dim, int tag, double angle, int *ierr);

RMSH_API int rmshPluginSetNumber(const char *name, const char *option, double value, int *ierr);
RMSH_API int rmshPluginSetString(const char *name, const char *option, const char *value, int *ierr);
RMSH_API int rmshPluginRun(const char *name, int *ierr);

RMSH_API int rmshGuiInitialize(int *ierr);
RMSH_API int rmshGuiRun(int *ierr);
RMSH_API int rmshGuiWait(double time, int *ierr);

#ifdef __cplusplus
}
#endif

#endif
