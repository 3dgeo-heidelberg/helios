#pragma once


#include "AbstractGeometryFilter.h"
#include <ogr_spatialref.h>
#include <gdal_priv.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Loader for tiff files
 */
class GeoTiffFileLoader : public AbstractGeometryFilter {

public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Source coordinate reference system
     */
    std::shared_ptr<OGRSpatialReference> sourceCRS;
    /**
     * @brief Tiff layer
     */
    OGRLayer *layer = nullptr;
    /**
     * @brief Tiff raster
     */
    GDALRasterBand *raster = nullptr;
    /**
     * @brief Tiff envelope
     */
    OGREnvelope *env = nullptr;
    /**
     * @brief Number of elements per raster along x axis
     */
    int rasterWidth;
    /**
     * @brief Number of elements per raster along y axis
     */
    int rasterHeight;
    /**
     * @brief Minimum value for x coordinate (starting point)
     */
    double minx;
    /**
     * @brief Minimum value for y coordinate (starting point)
     */
    double miny;
    /**
     * @brief Length of x axis
     */
    double width;
    /**
     * @brief Length of y axis
     */
    double height;
    /**
     * @brief Width divided by rasterWidth
     * @see GeoTiffFileLoader::Width
     * @see GeoTiffFileLoader::rasterWidth
     */
    double pixelWidth;
    /**
     * @brief Height divided by rasterHeight
     * @see GeoTiffFileLoader::Height
     * @see GeoTiffFileLoader::rasterHeight
     */
    double pixelHeight;
    /**
     * @brief Vertices from tiff. Obtain using the fillVertices function and
     * release using releaseVertices function.
     *
     * @see GeoTiffFileLoader::fillVertices
     * @see GeoTiffFileLoader::releaseVertices
     */
    Vertex ***vertices = nullptr;


    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Base constructor for tiff files loader
     * @see AbstractGeometryFilter::AbstractGeometryFilter(ScenePart*)
     */
	GeoTiffFileLoader() : AbstractGeometryFilter(new ScenePart()) {}

	// ***  R U N  *** //
	// *************** //
	/**
	 * @see AbstractGeometryFilter::run
	 */
	ScenePart* run();

	// ***  METHODS  *** //
	// ***************** //
	/**
	 * @brief Obtain coordinate reference system.
	 * @see GeoTiffFileLoader::sourceCRS
	 */
	void obtainCRS(GDALDataset *tiff);
	/**
	 * @brief Obtain layer from tiff
	 * @see GeoTiffFileLoader::layer
	 */
	void obtainLayer(GDALDataset *tiff);
	/**
	 * @brief Obtain raster from tiff
	 * @see GeoTiffFileLoader::raster
	 */
	void obtainRaster(GDALDataset *tiff);
	/**
	 * @brief Obtain envelope from tiff
	 * @see GeoTiffFileLoader::env
	 */
	void obtainEnvelope(GDALDataset *tiff);
	/**
	 * @brief Fill vertices with raster data.
	 * These vertices must be released with function releaseVertices
	 * @see GeoTiffFileLoader::vertices
	 * @see GeoTiffFileLoader::releaseVertices
	 */
	void fillVertices();
	/**
	 * @brief Release vertices previously obtained through fillVertices
	 * function
	 * @see GeoTiffFileLoader::fillVertices
	 */
    void releaseVertices();
	/**
	 * @brief Build triangles from vertices
	 */
	void buildTriangles();

};